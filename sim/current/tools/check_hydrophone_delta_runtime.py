#!/usr/bin/env python3
"""Validate unstamped hydrophone delta-range timing against MuJoCo ground truth."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String


def _correlation(lhs: list[float], rhs: list[float]) -> float | None:
    if len(lhs) != len(rhs) or len(lhs) < 3:
        return None
    left_mean = statistics.fmean(lhs)
    right_mean = statistics.fmean(rhs)
    numerator = sum((a - left_mean) * (b - right_mean) for a, b in zip(lhs, rhs))
    left_energy = sum((value - left_mean) ** 2 for value in lhs)
    right_energy = sum((value - right_mean) ** 2 for value in rhs)
    denominator = math.sqrt(left_energy * right_energy)
    return numerator / denominator if denominator > 1.0e-12 else None


class DeltaCheck(Node):
    def __init__(self) -> None:
        super().__init__("hydrophone_delta_runtime_check")
        self.started = time.monotonic()
        self.position: tuple[float, float, float] | None = None
        self.position_wall = float("-inf")
        self.pinger: tuple[float, float, float] | None = None
        self.hydrophone_true_range_m: float | None = None
        self.raw_window: deque[float] = deque(maxlen=3)
        self.cumulative = 0.0
        self.records: list[dict[str, object]] = []
        self.create_subscription(
            Odometry, "/sim/odom", self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            String,
            "/mujoco/course_buoys/status",
            self._on_buoys,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/mujoco/hydrophone/status",
            self._on_hydrophone_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            "/audio_phase_estimator/delta_range_m",
            self._on_delta,
            qos_profile_sensor_data,
        )

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.position = (float(position.x), float(position.y), float(position.z))
        self.position_wall = time.monotonic()

    def _on_buoys(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return

    def _on_hydrophone_status(self, message: String) -> None:
        """Read the MuJoCo oracle only in this offline contract checker.

        The test-tank pinger is a fixed acoustic site rather than a course
        buoy, so it is intentionally absent from /mujoco/course_buoys/status.
        The controller never subscribes to this diagnostic topic.
        """
        try:
            payload = json.loads(message.data)
            value = float(payload.get("range_m"))
        except (TypeError, ValueError):
            return
        if math.isfinite(value) and value > 0.0:
            self.hydrophone_true_range_m = value
        for buoy in payload.get("buoys", []):
            if "pinger" not in str(buoy.get("id", "")).lower():
                continue
            xyz = buoy.get("attach_xyz") or buoy.get("target_xyz")
            if isinstance(xyz, list) and len(xyz) >= 3:
                self.pinger = tuple(float(value) for value in xyz[:3])
            return

    def _on_delta(self, message: Float64) -> None:
        now = time.monotonic()
        delta = float(message.data)
        if (
            self.position is None
            or (self.pinger is None and self.hydrophone_true_range_m is None)
            or not math.isfinite(delta)
            or abs(delta) > 0.10
        ):
            return
        self.raw_window.append(delta)
        filtered = statistics.median(self.raw_window)
        self.cumulative += filtered
        true_range = (
            self.hydrophone_true_range_m
            if self.hydrophone_true_range_m is not None
            else math.sqrt(
                sum((target - vehicle) ** 2 for target, vehicle in zip(self.pinger, self.position))
            )
        )
        self.records.append(
            {
                "t_s": now - self.started,
                "position": list(self.position),
                "pinger": list(self.pinger) if self.pinger is not None else None,
                "odom_age_s": now - self.position_wall,
                "delta_range_m": delta,
                "filtered_delta_range_m": filtered,
                "cumulative_range_change_m": self.cumulative,
                "true_range_m": true_range,
            }
        )

    def summary(self) -> dict[str, object]:
        if len(self.records) < 4:
            return {"samples": len(self.records), "passed": False, "failure": "too few samples"}
        raw = [float(record["delta_range_m"]) for record in self.records]
        filtered = [float(record["filtered_delta_range_m"]) for record in self.records]
        ranges = [float(record["true_range_m"]) for record in self.records]
        times = [float(record["t_s"]) for record in self.records]
        true_delta = [ranges[index] - ranges[index - 1] for index in range(1, len(ranges))]

        shifts: dict[int, float | None] = {}
        for shift in range(-6, 7):
            estimates: list[float] = []
            truth: list[float] = []
            for index in range(1, len(filtered)):
                true_index = index + shift
                if 1 <= true_index < len(ranges):
                    estimates.append(filtered[index])
                    truth.append(ranges[true_index] - ranges[true_index - 1])
            shifts[shift] = _correlation(estimates, truth)
        finite_shifts = {
            shift: value for shift, value in shifts.items() if value is not None and math.isfinite(value)
        }
        best_shift = max(finite_shifts, key=finite_shifts.get) if finite_shifts else 0
        best_correlation = finite_shifts.get(best_shift)

        estimates = filtered[1:]
        correlation = _correlation(estimates, true_delta)
        estimate_energy = sum(value * value for value in estimates)
        scale = (
            sum(estimate * truth for estimate, truth in zip(estimates, true_delta))
            / estimate_energy
            if estimate_energy > 1.0e-12
            else None
        )
        moving_pairs = [
            (estimate, truth)
            for estimate, truth in zip(estimates, true_delta)
            if abs(truth) >= 5.0e-4
        ]
        sign_agreement = (
            sum((estimate >= 0.0) == (truth >= 0.0) for estimate, truth in moving_pairs)
            / len(moving_pairs)
            if moving_pairs
            else None
        )

        cumulative = [float(record["cumulative_range_change_m"]) for record in self.records]
        range_changes = [value - ranges[0] for value in ranges]
        residual = [truth - estimate for truth, estimate in zip(range_changes, cumulative)]
        time_mean = statistics.fmean(times)
        residual_mean = statistics.fmean(residual)
        time_energy = sum((value - time_mean) ** 2 for value in times)
        drift = (
            sum(
                (stamp - time_mean) * (value - residual_mean)
                for stamp, value in zip(times, residual)
            )
            / time_energy
            if time_energy > 1.0e-12
            else 0.0
        )
        intercept = residual_mean - drift * time_mean
        detrended = [
            value - (intercept + drift * stamp) for stamp, value in zip(times, residual)
        ]
        detrended_rms = math.sqrt(statistics.fmean(value * value for value in detrended))
        median_odom_age = statistics.median(
            float(record["odom_age_s"]) for record in self.records
        )
        duration = max(times[-1] - times[0], 1.0e-9)
        passed = bool(
            best_correlation is not None
            and best_correlation >= 0.80
            and abs(best_shift) <= 2
            and detrended_rms <= 0.25
        )
        return {
            "samples": len(self.records),
            "output_rate_hz": (len(self.records) - 1) / duration,
            "median_odom_age_s": median_odom_age,
            "same_index_correlation": correlation,
            "best_sample_shift": best_shift,
            "best_correlation": best_correlation,
            "delta_scale_true_per_estimate": scale,
            "moving_sign_agreement": sign_agreement,
            "cumulative_drift_mps": drift,
            "cumulative_detrended_rms_m": detrended_rms,
            "range_start_m": ranges[0],
            "range_final_m": ranges[-1],
            "passed": passed,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--trace", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = DeltaCheck()
    deadline = time.monotonic() + max(1.0, args.duration)
    try:
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        summary = node.summary()
        if args.trace is not None:
            args.trace.parent.mkdir(parents=True, exist_ok=True)
            args.trace.write_text(
                json.dumps({"summary": summary, "records": node.records}, indent=2) + "\n",
                encoding="utf-8",
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
