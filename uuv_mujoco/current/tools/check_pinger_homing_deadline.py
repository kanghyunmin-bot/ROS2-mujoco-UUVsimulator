#!/usr/bin/env python3
"""Time pinger homing with MuJoCo ground truth used only as a test oracle."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class DeadlineOracle(Node):
    def __init__(self, success_range_m: float, success_hold_s: float) -> None:
        super().__init__("pinger_homing_deadline_oracle")
        self.success_range_m = float(success_range_m)
        self.success_hold_s = float(success_hold_s)
        self.sim_time_s: float | None = None
        self.controller_active = False
        self.controller_mode = ""
        self.controller_profile = ""
        self.controller_direction_frame = ""
        self.controller_state = ""
        self.controller_states: list[str] = []
        self.controller_complete = False
        self.start_wall_s: float | None = None
        self.start_sim_s: float | None = None
        self.complete_wall_s: float | None = None
        self.complete_sim_s: float | None = None
        self.success_started_wall_s: float | None = None
        self.success_started_sim_s: float | None = None
        self.success_wall_s: float | None = None
        self.success_sim_s: float | None = None
        self.oracle_hold_verified = False
        self.oracle_hold_wall_s: float | None = None
        self.oracle_hold_sim_s: float | None = None
        self.oracle_hold_samples = 0
        self._current_hold_samples = 0
        self._pending_oracle_sample: tuple[
            float, float, float | None, list[float] | None, list[float] | None
        ] | None = None
        self.range_start_m: float | None = None
        self.range_min_m: float | None = None
        self.range_latest_m: float | None = None
        self.range_samples = 0
        self.invalid_numeric = False
        self.latest_direction_body: list[float] | None = None
        self.latest_direction_world: list[float] | None = None
        self.estimated_source_world: list[float] | None = None
        self.source_locked = False
        self.phase_rms_residual_m: float | None = None
        self.condition_number: float | None = None
        self.absolute_range_fit: dict[str, object] = {}

        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            String, "/mujoco/hydrophone/status", self._on_oracle, sensor_qos
        )
        self.create_subscription(
            String, "/pinger_homing/status", self._on_controller, 20
        )
        self.create_subscription(Clock, "/uuv_mujoco/clock", self._on_clock, 20)

    def _on_clock(self, message: Clock) -> None:
        self.sim_time_s = (
            float(message.clock.sec) + 1.0e-9 * float(message.clock.nanosec)
        )
        # The controller status and clock subscriptions are independent.  A live
        # status can therefore be dispatched first even though both publishers
        # were already running.  Anchor the simulation stopwatch on the first
        # clock received after that status instead of leaving start_sim_s unset.
        if self.start_wall_s is not None and self.start_sim_s is None:
            self.start_sim_s = self.sim_time_s
        if (
            self.success_started_wall_s is not None
            and self.success_started_sim_s is None
        ):
            self.success_started_sim_s = self.sim_time_s

    def _on_controller(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        state = str(payload.get("state", ""))
        self.controller_mode = str(payload.get("acoustic_estimator_mode", ""))
        self.controller_profile = str(payload.get("controller_profile", ""))
        self.controller_direction_frame = str(payload.get("direction_frame", ""))
        self.controller_state = state
        self.estimated_source_world = self._finite_vector(
            payload.get("estimated_source_world")
        )
        self.source_locked = bool(payload.get("source_locked", False))
        self.phase_rms_residual_m = self._finite_scalar(
            payload.get("rms_residual_m")
        )
        self.condition_number = self._finite_scalar(payload.get("condition_number"))
        absolute_fit = payload.get("absolute_range_fit")
        if isinstance(absolute_fit, dict):
            self.absolute_range_fit = dict(absolute_fit)
        if state and (not self.controller_states or self.controller_states[-1] != state):
            self.controller_states.append(state)
        now_wall_s = time.monotonic()
        active = bool(payload.get("control_output_active", False))
        self.controller_active = self.controller_active or active
        if active and self.start_wall_s is None:
            self.start_wall_s = now_wall_s
            self.start_sim_s = self.sim_time_s
            self._consume_pending_oracle(now_wall_s)
        if state == "COMPLETE" and not self.controller_complete:
            self.controller_complete = True
            if self.start_wall_s is not None:
                self.complete_wall_s = now_wall_s - self.start_wall_s
            if self.start_sim_s is not None and self.sim_time_s is not None:
                self.complete_sim_s = self.sim_time_s - self.start_sim_s

    @staticmethod
    def _finite_vector(value: object) -> list[float] | None:
        if not isinstance(value, list) or len(value) < 3:
            return None
        vector = [float(component) for component in value[:3]]
        return vector if all(math.isfinite(component) for component in vector) else None

    @staticmethod
    def _finite_scalar(value: object) -> float | None:
        try:
            scalar = float(value)
        except (TypeError, ValueError):
            return None
        return scalar if math.isfinite(scalar) else None

    def _on_oracle(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            range_m = float(payload["range_m"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        direction_body = self._finite_vector(payload.get("direction_body"))
        direction_world = self._finite_vector(payload.get("direction_world"))
        now_wall_s = time.monotonic()
        now_sim_s = self.sim_time_s
        if self.start_wall_s is None:
            # Retain one fresh sample so callback ordering at controller startup
            # cannot discard the beginning of a short success hold.
            if math.isfinite(range_m):
                self._pending_oracle_sample = (
                    range_m,
                    now_wall_s,
                    now_sim_s,
                    direction_body,
                    direction_world,
                )
            return
        self._record_oracle_sample(
            range_m, now_wall_s, now_sim_s, direction_body, direction_world
        )

    def _consume_pending_oracle(self, now_wall_s: float) -> None:
        pending = self._pending_oracle_sample
        self._pending_oracle_sample = None
        if pending is None:
            return
        range_m, sample_wall_s, sample_sim_s, direction_body, direction_world = pending
        if now_wall_s - sample_wall_s > 1.0:
            return
        # A pre-start callback is useful as the first boundary sample, but time
        # before live controller output must never count toward the hold.
        effective_wall_s = max(sample_wall_s, self.start_wall_s or sample_wall_s)
        effective_sim_s = sample_sim_s
        if self.start_sim_s is not None:
            effective_sim_s = max(sample_sim_s or self.start_sim_s, self.start_sim_s)
        self._record_oracle_sample(
            range_m,
            effective_wall_s,
            effective_sim_s,
            direction_body,
            direction_world,
        )

    def _record_oracle_sample(
        self,
        range_m: float,
        now_wall_s: float,
        now_sim_s: float | None,
        direction_body: list[float] | None,
        direction_world: list[float] | None,
    ) -> None:
        if not math.isfinite(range_m):
            self.invalid_numeric = True
            return
        self.latest_direction_body = direction_body
        self.latest_direction_world = direction_world
        if direction_body is None or direction_world is None:
            self.invalid_numeric = True
        self.range_samples += 1
        self.range_latest_m = range_m
        if self.range_start_m is None:
            self.range_start_m = range_m
        self.range_min_m = range_m if self.range_min_m is None else min(self.range_min_m, range_m)

        if range_m > self.success_range_m:
            if not self.oracle_hold_verified:
                self.success_started_wall_s = None
                self.success_started_sim_s = None
                self._current_hold_samples = 0
            return
        if self.oracle_hold_verified:
            return
        if self.success_started_wall_s is None:
            self.success_started_wall_s = now_wall_s
            self.success_started_sim_s = now_sim_s
            self._current_hold_samples = 1
            return
        self._current_hold_samples += 1
        wall_hold_s = now_wall_s - self.success_started_wall_s
        sim_hold_s: float | None = (
            now_sim_s - self.success_started_sim_s
            if now_sim_s is not None and self.success_started_sim_s is not None
            else None
        )
        if (
            self.start_wall_s is not None
            and self.start_sim_s is not None
            and sim_hold_s is not None
            and self._current_hold_samples >= 2
            and wall_hold_s >= self.success_hold_s
            and sim_hold_s >= self.success_hold_s
        ):
            # Latch the first verified crossing.  Continuing to overwrite these
            # values made a completed run appear slower on every later sample.
            self.oracle_hold_verified = True
            self.oracle_hold_wall_s = wall_hold_s
            self.oracle_hold_sim_s = sim_hold_s
            self.oracle_hold_samples = self._current_hold_samples
            self.success_wall_s = now_wall_s - self.start_wall_s
            self.success_sim_s = now_sim_s - self.start_sim_s

    def summary(self) -> dict[str, object]:
        progress = (
            self.range_start_m - self.range_min_m
            if self.range_start_m is not None and self.range_min_m is not None
            else None
        )
        return {
            "controller_active": self.controller_active,
            "controller_mode": self.controller_mode,
            "controller_profile": self.controller_profile,
            "controller_direction_frame": self.controller_direction_frame,
            "controller_state": self.controller_state,
            "controller_states": self.controller_states,
            "controller_complete": self.controller_complete,
            "complete_wall_s": self.complete_wall_s,
            "complete_sim_s": self.complete_sim_s,
            "success_range_m": self.success_range_m,
            "success_hold_s": self.success_hold_s,
            "success_wall_s": self.success_wall_s,
            "success_sim_s": self.success_sim_s,
            "oracle_hold_verified": self.oracle_hold_verified,
            "oracle_hold_wall_s": self.oracle_hold_wall_s,
            "oracle_hold_sim_s": self.oracle_hold_sim_s,
            "oracle_hold_samples": self.oracle_hold_samples,
            "range_start_m": self.range_start_m,
            "range_min_m": self.range_min_m,
            "range_latest_m": self.range_latest_m,
            "range_progress_m": progress,
            "range_samples": self.range_samples,
            "direction_body": self.latest_direction_body,
            "direction_world": self.latest_direction_world,
            "estimated_source_world": self.estimated_source_world,
            "source_locked": self.source_locked,
            "phase_rms_residual_m": self.phase_rms_residual_m,
            "condition_number": self.condition_number,
            "absolute_range_fit": self.absolute_range_fit,
            "invalid_numeric": self.invalid_numeric,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("phase", "snr"), required=True)
    parser.add_argument(
        "--expected-controller-profile",
        default="real",
        choices=("real", "sim_fast"),
        help="Profile expected on the controller under test; real-parity GUI runs use real.",
    )
    parser.add_argument("--deadline-wall-s", type=float, default=65.0)
    parser.add_argument("--deadline-sim-s", type=float, default=65.0)
    parser.add_argument("--success-range-m", type=float, default=1.5)
    parser.add_argument("--success-hold-s", type=float, default=0.8)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = DeadlineOracle(args.success_range_m, args.success_hold_s)
    launched_wall_s = time.monotonic()
    timed_out = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.oracle_hold_verified and node.controller_complete:
                break
            wall_reference = node.start_wall_s or launched_wall_s
            if time.monotonic() - wall_reference > max(1.0, args.deadline_wall_s):
                timed_out = True
                break
            if (
                node.start_sim_s is not None
                and node.sim_time_s is not None
                and node.sim_time_s - node.start_sim_s > max(1.0, args.deadline_sim_s)
            ):
                timed_out = True
                break
        summary = node.summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    failures: list[str] = []
    if not summary["controller_active"]:
        failures.append("controller never produced live output")
    if summary["controller_mode"] != args.mode:
        failures.append(
            f"controller mode {summary['controller_mode']!r} != requested {args.mode!r}"
        )
    if summary["controller_profile"] != args.expected_controller_profile:
        failures.append(
            "controller profile "
            f"{summary['controller_profile']!r} != {args.expected_controller_profile!r}"
        )
    expected_frame = "body" if args.mode == "snr" else "world"
    if summary["controller_direction_frame"] != expected_frame:
        failures.append(
            "controller direction frame "
            f"{summary['controller_direction_frame']!r} != {expected_frame!r}"
        )
    if not summary["controller_complete"]:
        failures.append(
            f"controller did not report COMPLETE (last={summary['controller_state']!r})"
        )
    if summary["range_samples"] == 0:
        failures.append("no MuJoCo oracle range samples")
    if summary["invalid_numeric"]:
        failures.append("non-finite oracle value observed")
    if not summary["oracle_hold_verified"]:
        failures.append("oracle success range was not held before the deadline")
    elif (
        summary["oracle_hold_samples"] < 2
        or float(summary["oracle_hold_wall_s"]) < args.success_hold_s
        or float(summary["oracle_hold_sim_s"]) < args.success_hold_s
    ):
        failures.append("oracle hold evidence is incomplete")
    elif summary["success_wall_s"] is None or summary["success_sim_s"] is None:
        failures.append("oracle success timestamp is incomplete")
    elif float(summary["success_wall_s"]) > args.deadline_wall_s:
        failures.append("wall-time deadline exceeded")
    elif float(summary["success_sim_s"]) > args.deadline_sim_s:
        failures.append("simulation-time deadline exceeded")
    summary.update(
        {
            "requested_mode": args.mode,
            "expected_controller_profile": args.expected_controller_profile,
            "deadline_wall_s": args.deadline_wall_s,
            "deadline_sim_s": args.deadline_sim_s,
            "timed_out": timed_out,
            "effective_rtf": (
                float(summary["success_sim_s"]) / float(summary["success_wall_s"])
                if summary["success_sim_s"] is not None
                and summary["success_wall_s"] not in (None, 0.0)
                else None
            ),
            "failures": failures,
            "result": "PASS" if not failures else "FAIL",
        }
    )
    text = json.dumps(summary, indent=2, sort_keys=True)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
