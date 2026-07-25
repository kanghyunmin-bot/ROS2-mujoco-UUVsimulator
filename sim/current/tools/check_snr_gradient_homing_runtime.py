#!/usr/bin/env python3
"""Drive the SNR estimator with a known trajectory and verify convergence."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from audio_common_msgs.msg import Float64Stamped
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class SnrGradientRuntimeCheck(Node):
    def __init__(self, source_x: float, source_y: float) -> None:
        super().__init__("snr_gradient_homing_runtime_check")
        self.source_x = source_x
        self.source_y = source_y
        self.position = (0.0, 0.0)
        self.ready = False
        self.direction_samples: list[tuple[float, float, float]] = []
        self.confidence_samples: list[float] = []
        self.odom_pub = self.create_publisher(Odometry, "/sim/odom", 10)
        self.snr_pub = self.create_publisher(
            Float64Stamped, "/audio_phase_estimator/iq_snr_ratio_stamped", 10
        )
        self.create_subscription(
            Vector3Stamped, "/homing/direction", self._on_direction, 10
        )
        self.create_subscription(
            Float64, "/homing/snr_confidence", self._on_confidence, 10
        )
        self.create_subscription(Bool, "/homing/estimator_ready", self._on_ready, 10)

    def _on_direction(self, message: Vector3Stamped) -> None:
        if message.header.frame_id != "base_link":
            return
        vector = (float(message.vector.x), float(message.vector.y), float(message.vector.z))
        if all(math.isfinite(value) for value in vector):
            self.direction_samples.append(vector)

    def _on_confidence(self, message: Float64) -> None:
        if math.isfinite(message.data):
            self.confidence_samples.append(float(message.data))

    def _on_ready(self, message: Bool) -> None:
        self.ready = bool(message.data)

    def publish_sample(self, x: float, y: float, snr: float) -> None:
        self.position = (x, y)
        stamp = self.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.position.z = -1.0
        odometry.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odometry)
        rclpy.spin_once(self, timeout_sec=0.008)
        measurement = Float64Stamped()
        measurement.header.stamp = stamp
        measurement.header.frame_id = "hydrophone"
        measurement.data = snr
        self.snr_pub.publish(measurement)

    def current_true_direction(self) -> tuple[float, float]:
        dx = self.source_x - self.position[0]
        dy = self.source_y - self.position[1]
        norm = math.hypot(dx, dy)
        return dx / norm, dy / norm

    def summary(self) -> dict[str, object]:
        latest = self.direction_samples[-1] if self.direction_samples else None
        dot = None
        angle_deg = None
        if latest is not None:
            norm = math.hypot(latest[0], latest[1])
            if norm > 1.0e-9:
                truth = self.current_true_direction()
                dot = max(
                    -1.0,
                    min(1.0, latest[0] / norm * truth[0] + latest[1] / norm * truth[1]),
                )
                angle_deg = math.degrees(math.acos(dot))
        return {
            "ready": self.ready,
            "direction_samples": len(self.direction_samples),
            "confidence_samples": len(self.confidence_samples),
            "latest_confidence": (
                self.confidence_samples[-1] if self.confidence_samples else None
            ),
            "latest_direction": latest,
            "truth_direction": self.current_true_direction(),
            "direction_dot": dot,
            "angle_error_deg": angle_deg,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--publish-hz", type=float, default=20.0)
    parser.add_argument("--source-x", type=float, default=6.0)
    parser.add_argument("--source-y", type=float, default=2.0)
    parser.add_argument("--max-angle-error-deg", type=float, default=35.0)
    parser.add_argument("--min-confidence", type=float, default=0.10)
    args = parser.parse_args()

    rclpy.init()
    node = SnrGradientRuntimeCheck(args.source_x, args.source_y)
    period = 1.0 / max(args.publish_hz, 1.0)
    steps = max(24, int(max(args.duration, 1.0) / period))
    start = time.monotonic()
    try:
        for index in range(steps):
            phase = 2.0 * math.pi * 1.75 * index / max(steps - 1, 1)
            radius_scale = 0.55 + 0.45 * index / max(steps - 1, 1)
            x = 1.35 * radius_scale * math.cos(phase)
            y = 0.90 * radius_scale * math.sin(phase)
            distance = math.sqrt(
                (args.source_x - x) ** 2 + (args.source_y - y) ** 2 + 0.35**2
            )
            snr = 1.0 + 18.0 / distance
            node.publish_sample(x, y, snr)
            deadline = start + (index + 1) * period
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.01, deadline - time.monotonic()))
        settle_deadline = time.monotonic() + 0.5
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        summary = node.summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    failures: list[str] = []
    if not summary["ready"]:
        failures.append("estimator never became ready")
    if int(summary["direction_samples"]) == 0:
        failures.append("no /homing/direction samples")
    angle_error = summary["angle_error_deg"]
    if angle_error is None or float(angle_error) > args.max_angle_error_deg:
        failures.append(
            f"angle error {angle_error} > {args.max_angle_error_deg} degrees"
        )
    confidence = summary["latest_confidence"]
    if confidence is None or float(confidence) < args.min_confidence:
        failures.append(f"confidence {confidence} < {args.min_confidence}")
    summary["failures"] = failures
    summary["result"] = "FAIL" if failures else "PASS"
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
