#!/usr/bin/env python3
"""Verify the SNR homing mode against MuJoCo diagnostics used only as an oracle."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


class SnrSimRuntimeCheck(Node):
    def __init__(self) -> None:
        super().__init__("snr_homing_sim_runtime_check")
        self.states: list[str] = []
        self.latest_state = ""
        self.ready_seen = False
        self.snr_values: list[float] = []
        self.confidences: list[float] = []
        self.ranges_m: list[float] = []
        self.horizontal_ranges_m: list[float] = []
        self.vertical_separations_m: list[float] = []
        self.direction_errors_deg: list[float] = []
        self.direction_errors_3d_deg: list[float] = []
        self.latest_truth_body: tuple[float, float, float] | None = None
        self.rc_active_samples = 0
        self.heave_active_samples = 0
        self.connected_seen = False
        self.armed_seen = False
        self.invalid_numeric = False
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/homing/control_state", self._on_state, state_qos
        )
        self.create_subscription(Bool, "/homing/estimator_ready", self._on_ready, 10)
        self.create_subscription(
            Float64, "/audio_phase_estimator/iq_snr_ratio", self._on_snr, 10
        )
        self.create_subscription(
            Float64, "/homing/snr_confidence", self._on_confidence, 10
        )
        self.create_subscription(
            String, "/mujoco/hydrophone/status", self._on_oracle_status, 10
        )
        self.create_subscription(
            Vector3Stamped, "/homing/direction", self._on_direction, 10
        )
        self.create_subscription(
            OverrideRCIn, "/mavros/rc/override", self._on_rc, 10
        )
        self.create_subscription(State, "/mavros/state", self._on_vehicle_state, 10)

    def _on_state(self, message: String) -> None:
        state = str(message.data).strip()
        self.latest_state = state
        if state and (not self.states or self.states[-1] != state):
            self.states.append(state)

    def _on_ready(self, message: Bool) -> None:
        self.ready_seen = self.ready_seen or bool(message.data)

    def _append_finite(self, values: list[float], value: float) -> None:
        if math.isfinite(value):
            values.append(value)
        else:
            self.invalid_numeric = True

    def _on_snr(self, message: Float64) -> None:
        self._append_finite(self.snr_values, float(message.data))

    def _on_confidence(self, message: Float64) -> None:
        self._append_finite(self.confidences, float(message.data))

    def _on_oracle_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            range_m = float(payload["range_m"])
            direction = tuple(float(value) for value in payload["direction_body"][:3])
            direction_world = tuple(
                float(value) for value in payload["direction_world"][:3]
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not math.isfinite(range_m) or not all(math.isfinite(value) for value in direction):
            self.invalid_numeric = True
            return
        self.ranges_m.append(range_m)
        self.horizontal_ranges_m.append(range_m * math.hypot(direction[0], direction[1]))
        self.vertical_separations_m.append(abs(range_m * direction_world[2]))
        self.latest_truth_body = direction

    def _on_direction(self, message: Vector3Stamped) -> None:
        if (
            self.latest_state != "HOMING"
            or message.header.frame_id != "base_link"
            or self.latest_truth_body is None
        ):
            return
        estimate = (float(message.vector.x), float(message.vector.y), float(message.vector.z))
        truth = self.latest_truth_body
        estimate_norm = math.hypot(estimate[0], estimate[1])
        truth_norm = math.hypot(truth[0], truth[1])
        if estimate_norm <= 1.0e-9 or truth_norm <= 1.0e-9:
            return
        dot = max(
            -1.0,
            min(
                1.0,
                estimate[0] / estimate_norm * truth[0] / truth_norm
                + estimate[1] / estimate_norm * truth[1] / truth_norm,
            ),
        )
        self.direction_errors_deg.append(math.degrees(math.acos(dot)))
        estimate_norm_3d = math.sqrt(sum(value * value for value in estimate))
        truth_norm_3d = math.sqrt(sum(value * value for value in truth))
        if estimate_norm_3d > 1.0e-9 and truth_norm_3d > 1.0e-9:
            dot_3d = max(
                -1.0,
                min(
                    1.0,
                    sum(
                        estimate[index] / estimate_norm_3d
                        * truth[index] / truth_norm_3d
                        for index in range(3)
                    ),
                ),
            )
            self.direction_errors_3d_deg.append(math.degrees(math.acos(dot_3d)))

    def _on_rc(self, message: OverrideRCIn) -> None:
        channels = list(message.channels)
        if len(channels) >= 5 and any(
            value not in {
                OverrideRCIn.CHAN_RELEASE,
                OverrideRCIn.CHAN_NOCHANGE,
                1500,
            }
            for value in channels[2:5]
        ):
            self.rc_active_samples += 1
        if len(channels) >= 3 and channels[2] not in {
            OverrideRCIn.CHAN_RELEASE,
            OverrideRCIn.CHAN_NOCHANGE,
            1500,
        }:
            self.heave_active_samples += 1

    def _on_vehicle_state(self, message: State) -> None:
        self.connected_seen = self.connected_seen or bool(message.connected)
        self.armed_seen = self.armed_seen or bool(message.armed)

    @staticmethod
    def _progress(values: list[float]) -> float | None:
        if not values:
            return None
        return values[0] - min(values)

    def summary(self) -> dict[str, object]:
        return {
            "states": self.states,
            "latest_state": self.latest_state,
            "ready_seen": self.ready_seen,
            "connected_seen": self.connected_seen,
            "armed_seen": self.armed_seen,
            "snr_samples": len(self.snr_values),
            "snr_min": min(self.snr_values) if self.snr_values else None,
            "snr_max": max(self.snr_values) if self.snr_values else None,
            "confidence_samples": len(self.confidences),
            "confidence_latest": self.confidences[-1] if self.confidences else None,
            "range_start_m": self.ranges_m[0] if self.ranges_m else None,
            "range_min_m": min(self.ranges_m) if self.ranges_m else None,
            "range_progress_m": self._progress(self.ranges_m),
            "horizontal_range_start_m": (
                self.horizontal_ranges_m[0] if self.horizontal_ranges_m else None
            ),
            "horizontal_range_min_m": (
                min(self.horizontal_ranges_m) if self.horizontal_ranges_m else None
            ),
            "horizontal_range_progress_m": self._progress(self.horizontal_ranges_m),
            "vertical_separation_start_m": (
                self.vertical_separations_m[0]
                if self.vertical_separations_m
                else None
            ),
            "vertical_separation_min_m": (
                min(self.vertical_separations_m)
                if self.vertical_separations_m
                else None
            ),
            "vertical_separation_progress_m": self._progress(
                self.vertical_separations_m
            ),
            "direction_error_median_deg": (
                statistics.median(self.direction_errors_deg)
                if self.direction_errors_deg
                else None
            ),
            "direction_error_samples": len(self.direction_errors_deg),
            "direction_error_3d_median_deg": (
                statistics.median(self.direction_errors_3d_deg)
                if self.direction_errors_3d_deg
                else None
            ),
            "direction_error_3d_samples": len(self.direction_errors_3d_deg),
            "rc_active_samples": self.rc_active_samples,
            "heave_active_samples": self.heave_active_samples,
            "invalid_numeric": self.invalid_numeric,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=75.0)
    parser.add_argument("--min-horizontal-progress-m", type=float, default=1.0)
    parser.add_argument("--max-direction-error-deg", type=float, default=70.0)
    parser.add_argument("--min-vertical-progress-m", type=float, default=0.0)
    parser.add_argument("--max-3d-direction-error-deg", type=float, default=180.0)
    parser.add_argument("--require-heave", action="store_true")
    parser.add_argument("--require-homing-state", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = SnrSimRuntimeCheck()
    deadline = time.monotonic() + max(1.0, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.latest_state == "ARRIVED":
                break
        summary = node.summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    failures: list[str] = []
    if not summary["connected_seen"]:
        failures.append("MAVROS never connected")
    if not summary["armed_seen"]:
        failures.append("vehicle never armed")
    if not summary["ready_seen"]:
        failures.append("SNR estimator never became ready")
    if int(summary["snr_samples"]) == 0:
        failures.append("no PCM-derived SNR samples")
    if int(summary["rc_active_samples"]) == 0:
        failures.append("no active SNR controller RC samples")
    if args.require_homing_state and "HOMING" not in summary["states"]:
        failures.append("controller never entered HOMING")
    progress = summary["horizontal_range_progress_m"]
    if progress is None or float(progress) < args.min_horizontal_progress_m:
        failures.append(
            f"horizontal progress {progress} < {args.min_horizontal_progress_m} m"
        )
    direction_error = summary["direction_error_median_deg"]
    if direction_error is None or float(direction_error) > args.max_direction_error_deg:
        failures.append(
            f"direction error {direction_error} > {args.max_direction_error_deg} degrees"
        )
    vertical_progress = summary["vertical_separation_progress_m"]
    if (
        vertical_progress is None
        or float(vertical_progress) < args.min_vertical_progress_m
    ):
        failures.append(
            f"vertical progress {vertical_progress} < {args.min_vertical_progress_m} m"
        )
    direction_error_3d = summary["direction_error_3d_median_deg"]
    if (
        direction_error_3d is None
        or float(direction_error_3d) > args.max_3d_direction_error_deg
    ):
        failures.append(
            f"3D direction error {direction_error_3d} > "
            f"{args.max_3d_direction_error_deg} degrees"
        )
    if args.require_heave and int(summary["heave_active_samples"]) == 0:
        failures.append("no active vertical RC samples")
    if summary["invalid_numeric"]:
        failures.append("non-finite runtime value observed")
    summary["failures"] = failures
    summary["result"] = "FAIL" if failures else "PASS"
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
