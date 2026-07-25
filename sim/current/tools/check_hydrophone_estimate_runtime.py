#!/usr/bin/env python3
"""Measure hydrophone/FSM direction accuracy against MuJoCo ground truth."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def _unit(vector: tuple[float, float, float]) -> tuple[float, float, float] | None:
    norm = math.sqrt(sum(value * value for value in vector))
    if not math.isfinite(norm) or norm < 1.0e-9:
        return None
    return tuple(value / norm for value in vector)


def _angle_deg(
    lhs: tuple[float, float, float], rhs: tuple[float, float, float]
) -> float | None:
    left = _unit(lhs)
    right = _unit(rhs)
    if left is None or right is None:
        return None
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(left, right))))
    return math.degrees(math.acos(dot))


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


class RuntimeCheck(Node):
    def __init__(self, direction_warmup_s: float) -> None:
        super().__init__("hydrophone_estimate_runtime_check")
        self.direction_warmup_s = direction_warmup_s
        self.position: tuple[float, float, float] | None = None
        self.yaw_rad = 0.0
        self.pinger_position: tuple[float, float, float] | None = None
        self.pinger_detached = False
        self.first_direction_wall: float | None = None
        self.upstream_errors_deg: list[float] = []
        self.body_errors_deg: list[float] = []
        self.dynamic_body_errors_deg: list[float] = []
        self.phase_fit_errors_m: list[float] = []
        self.true_ranges_m: list[float] = []
        self.direction_frames: set[str] = set()
        self.hydrophone_sources: set[str] = set()
        self.states: list[str] = []
        self.latest_state = ""
        self.latest_failure = ""
        self.latest_source = ""

        self.create_subscription(
            Odometry, "/sim/odom", self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/mujoco/course_buoys/status", self._on_buoys, qos_profile_sensor_data
        )
        self.create_subscription(
            Vector3Stamped,
            "/homing/direction",
            self._on_upstream_direction,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3Stamped,
            "/mission/hydrophone/direction_body",
            self._on_body_direction,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "/mission/fsm/status", self._on_status, 10)

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.position = (float(position.x), float(position.y), float(position.z))
        self.yaw_rad = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        true_world = self._true_world_direction()
        if true_world is not None and self.first_direction_wall is not None:
            self.true_ranges_m.append(
                math.sqrt(sum(value * value for value in true_world))
            )

    def _on_buoys(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        for buoy in payload.get("buoys", []):
            if "pinger" not in str(buoy.get("id", "")).lower():
                continue
            position = buoy.get("attach_xyz") or buoy.get("target_xyz")
            if isinstance(position, list) and len(position) >= 3:
                self.pinger_position = tuple(float(value) for value in position[:3])
            self.pinger_detached = bool(buoy.get("detached", False))
            return

    def _on_upstream_direction(self, message: Vector3Stamped) -> None:
        now = time.monotonic()
        if self.first_direction_wall is None:
            self.first_direction_wall = now
        self.direction_frames.add(str(message.header.frame_id))
        if now - self.first_direction_wall < self.direction_warmup_s:
            return
        true_world = self._true_world_direction()
        if true_world is None:
            return
        estimate = (message.vector.x, message.vector.y, message.vector.z)
        error = _angle_deg(estimate, true_world)
        if error is not None:
            self.upstream_errors_deg.append(error)

    def _on_body_direction(self, message: Vector3Stamped) -> None:
        if self.first_direction_wall is None:
            return
        if time.monotonic() - self.first_direction_wall < self.direction_warmup_s:
            return
        true_world = self._true_world_direction()
        if true_world is None:
            return
        cosine = math.cos(self.yaw_rad)
        sine = math.sin(self.yaw_rad)
        true_body = (
            cosine * true_world[0] + sine * true_world[1],
            -sine * true_world[0] + cosine * true_world[1],
            true_world[2],
        )
        estimate = (message.vector.x, message.vector.y, message.vector.z)
        error = _angle_deg(estimate, true_body)
        if error is not None:
            self.body_errors_deg.append(error)
            if self.latest_source in {
                "phase_range_position_fusion",
                "acoustic_position_fusion",
            }:
                self.dynamic_body_errors_deg.append(error)

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = str(payload.get("state", ""))
        if state and state != self.latest_state:
            self.states.append(state)
            self.latest_state = state
        source = str(payload.get("hydrophone_source", ""))
        if source:
            self.hydrophone_sources.add(source)
            self.latest_source = source
        phase_fit = payload.get("phase_range_position_fit_xy")
        if (
            source == "phase_range_position_fusion"
            and self.pinger_position is not None
            and isinstance(phase_fit, list)
            and len(phase_fit) >= 2
        ):
            try:
                error_m = math.hypot(
                    float(phase_fit[0]) - self.pinger_position[0],
                    float(phase_fit[1]) - self.pinger_position[1],
                )
            except (TypeError, ValueError):
                pass
            else:
                if math.isfinite(error_m):
                    self.phase_fit_errors_m.append(error_m)
        self.latest_failure = str(payload.get("failure", ""))

    def _true_world_direction(self) -> tuple[float, float, float] | None:
        if self.position is None or self.pinger_position is None:
            return None
        return tuple(
            target - vehicle for target, vehicle in zip(self.pinger_position, self.position)
        )

    def summary(self) -> dict[str, object]:
        ranges = self.true_ranges_m
        return {
            "direction_frames": sorted(self.direction_frames),
            "hydrophone_sources": sorted(self.hydrophone_sources),
            "states": self.states,
            "latest_state": self.latest_state,
            "failure": self.latest_failure,
            "pinger_detached": self.pinger_detached,
            "upstream_samples": len(self.upstream_errors_deg),
            "upstream_angle_median_deg": (
                statistics.median(self.upstream_errors_deg)
                if self.upstream_errors_deg
                else None
            ),
            "upstream_angle_p95_deg": _percentile(self.upstream_errors_deg, 0.95),
            "body_arrow_samples": len(self.body_errors_deg),
            "body_arrow_angle_median_deg": (
                statistics.median(self.body_errors_deg) if self.body_errors_deg else None
            ),
            "dynamic_body_arrow_samples": len(self.dynamic_body_errors_deg),
            "dynamic_body_arrow_angle_median_deg": (
                statistics.median(self.dynamic_body_errors_deg)
                if self.dynamic_body_errors_deg
                else None
            ),
            "phase_fit_error_median_m": (
                statistics.median(self.phase_fit_errors_m)
                if self.phase_fit_errors_m
                else None
            ),
            "range_start_m": ranges[0] if ranges else None,
            "range_min_m": min(ranges) if ranges else None,
            "range_final_m": ranges[-1] if ranges else None,
            "range_progress_m": (ranges[0] - min(ranges)) if ranges else None,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--direction-warmup", type=float, default=3.0)
    parser.add_argument("--max-upstream-median-angle-deg", type=float, default=180.0)
    parser.add_argument("--max-control-median-angle-deg", type=float, default=45.0)
    parser.add_argument("--max-phase-position-error-m", type=float, default=3.0)
    parser.add_argument("--min-range-progress-m", type=float, default=1.0)
    parser.add_argument("--require-dynamic-fusion", action="store_true")
    parser.add_argument("--require-detach", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = RuntimeCheck(max(0.0, args.direction_warmup))
    deadline = time.monotonic() + max(1.0, args.duration)
    try:
        try:
            while rclpy.ok() and time.monotonic() < deadline and not node.pinger_detached:
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        summary = node.summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    failures: list[str] = []
    median_error = summary["upstream_angle_median_deg"]
    if summary["upstream_samples"] == 0:
        failures.append("no upstream /homing/direction samples")
    elif median_error is None or float(median_error) > args.max_upstream_median_angle_deg:
        failures.append(
            "upstream median angle "
            f"{median_error} > {args.max_upstream_median_angle_deg} deg"
        )
    if "upstream_ekf" not in summary["hydrophone_sources"]:
        failures.append("FSM did not report upstream_ekf as its direction source")
    unexpected_sources = set(summary["hydrophone_sources"]) - {
        "acoustic_position_fusion",
        "phase_range_position_fusion",
        "upstream_ekf",
        "unavailable",
    }
    if unexpected_sources:
        failures.append(f"FSM used unsupported direction sources: {sorted(unexpected_sources)}")
    dynamic_sources = {"phase_range_position_fusion", "acoustic_position_fusion"}
    if args.require_dynamic_fusion and not (
        dynamic_sources & set(summary["hydrophone_sources"])
    ):
        failures.append("FSM never activated a dynamic pinger-position fusion source")
    phase_position_error = summary["phase_fit_error_median_m"]
    if (
        "phase_range_position_fusion" in summary["hydrophone_sources"]
        and (
            phase_position_error is None
            or float(phase_position_error) > args.max_phase_position_error_m
        )
    ):
        failures.append(
            "phase-position median error "
            f"{phase_position_error} > {args.max_phase_position_error_m} m"
        )
    control_sample_key = (
        "dynamic_body_arrow_samples" if args.require_dynamic_fusion else "body_arrow_samples"
    )
    control_median_key = (
        "dynamic_body_arrow_angle_median_deg"
        if args.require_dynamic_fusion
        else "body_arrow_angle_median_deg"
    )
    control_median = summary[control_median_key]
    if summary[control_sample_key] == 0:
        failures.append("no body-frame control arrow samples")
    elif control_median is None or float(control_median) > args.max_control_median_angle_deg:
        failures.append(
            "control direction median angle "
            f"{control_median} > {args.max_control_median_angle_deg} deg"
        )
    progress = summary["range_progress_m"]
    if progress is None or float(progress) < args.min_range_progress_m:
        failures.append(f"range progress {progress} < {args.min_range_progress_m} m")
    if args.require_detach and not summary["pinger_detached"]:
        failures.append("pinger did not detach")
    summary["passed"] = not failures
    summary["failures"] = failures
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
