#!/usr/bin/env python3
"""Compare the real-contract MAVROS IMU attitude with MuJoCo ground truth.

Ground truth is used only as a simulator test oracle.  The deployed homing
controller continues to consume ``/mavros/imu/data`` and never subscribes to
the oracle topic.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def _stamp_seconds(msg: object) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def _yaw(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class AttitudeOracle(Node):
    def __init__(self, max_stamp_skew_s: float) -> None:
        super().__init__("mavros_imu_ground_truth_attitude_oracle")
        self.max_stamp_skew_s = max_stamp_skew_s
        self.latest_imu: Imu | None = None
        self.yaw_errors: list[float] = []
        self.create_subscription(
            Imu, "/mavros/imu/data", self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped,
            "/mujoco/ground_truth/pose",
            self._on_ground_truth,
            qos_profile_sensor_data,
        )

    def _on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg

    def _on_ground_truth(self, msg: PoseStamped) -> None:
        imu = self.latest_imu
        if imu is None:
            return
        if abs(_stamp_seconds(imu) - _stamp_seconds(msg)) > self.max_stamp_skew_s:
            return
        error = _wrap(_yaw(imu.orientation) - _yaw(msg.pose.orientation))
        self.yaw_errors.append(error)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--max-stamp-skew", type=float, default=0.10)
    parser.add_argument("--max-median-error-deg", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = AttitudeOracle(max(0.001, args.max_stamp_skew))
    deadline = time.monotonic() + max(0.1, args.duration)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.yaw_errors:
        print("mavros_imu_ground_truth_attitude=FAIL reason=no_synchronized_samples")
        return 2
    median_deg = math.degrees(statistics.median(node.yaw_errors))
    max_abs_deg = max(abs(math.degrees(value)) for value in node.yaw_errors)
    passed = abs(median_deg) <= args.max_median_error_deg
    print(
        "mavros_imu_ground_truth_attitude=" + ("PASS" if passed else "FAIL")
        + f" samples={len(node.yaw_errors)} median_yaw_error_deg={median_deg:.3f}"
        + f" max_abs_yaw_error_deg={max_abs_deg:.3f}"
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
