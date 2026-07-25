#!/usr/bin/env python3
"""Verify that exceeding the vehicle depth limit terminates RC control."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time

os.environ.setdefault("ROS_DOMAIN_ID", "196")

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from std_msgs.msg import String


class DepthLimitProbe(Node):
    def __init__(self) -> None:
        super().__init__("depth_limit_failsafe_runtime_probe")
        self.depth_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/test/depth_limit/depth", 10
        )
        self.status: dict = {}
        self.rc: OverrideRCIn | None = None
        self.create_subscription(
            String, "/test/depth_limit/status", self._on_status, 10
        )
        self.create_subscription(
            OverrideRCIn, "/test/depth_limit/rc", self._on_rc, 10
        )

    def _on_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self.status = value

    def _on_rc(self, message: OverrideRCIn) -> None:
        self.rc = message

    def publish_excess_depth(self) -> None:
        depth = PoseWithCovarianceStamped()
        depth.header.stamp = self.get_clock().now().to_msg()
        depth.pose.pose.position.z = -2.1
        depth.pose.pose.orientation.w = 1.0
        self.depth_pub.publish(depth)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    args = parser.parse_args()
    process = subprocess.Popen(
        [
            args.controller,
            "--ros-args",
            "-r",
            "__node:=test_depth_limit_failsafe_controller",
            "-p",
            "dry_run:=false",
            "-p",
            "navigation_mode:=no_odom_phase",
            "-p",
            "depth_pose_topic:=/test/depth_limit/depth",
            "-p",
            "status_topic:=/test/depth_limit/status",
            "-p",
            "rc_output_topic:=/test/depth_limit/rc",
            "-p",
            "max_vehicle_depth_m:=2.0",
            "-p",
            "depth_pose_timeout_s:=0.5",
            "-p",
            "max_runtime_s:=0.0",
        ]
    )
    rclpy.init()
    probe = DepthLimitProbe()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            probe.publish_excess_depth()
            rclpy.spin_once(probe, timeout_sec=0.04)
            if (
                probe.status.get("state") == "FAILED_DEPTH"
                and probe.rc is not None
                and all(
                    value == OverrideRCIn.CHAN_RELEASE
                    for value in probe.rc.channels
                )
            ):
                print("depth_limit_failsafe_runtime=PASS state=FAILED_DEPTH rc=RELEASE")
                return 0
        raise AssertionError(
            f"depth limit did not terminate with RC release: "
            f"status={probe.status} rc={probe.rc}"
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
