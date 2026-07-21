#!/usr/bin/env python3
"""Verify the physical XY/ALT_HOLD Phase profile does not depend on Bar30."""

from __future__ import annotations

import argparse
import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String


class DepthlessPhaseInputs(Node):
    def __init__(self) -> None:
        super().__init__("depthless_phase_inputs")
        self.imu_pub = self.create_publisher(Imu, "/test/depthless/imu", 20)
        self.delta_pub = self.create_publisher(Float64, "/test/depthless/delta", 50)
        self.status: dict = {}
        self.create_subscription(String, "/test/depthless/status", self._on_status, 20)

    def _on_status(self, message: String) -> None:
        try:
            candidate = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(candidate, dict):
            self.status = candidate

    def publish(self) -> None:
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.orientation.w = 1.0
        self.imu_pub.publish(imu)
        self.delta_pub.publish(Float64(data=0.001))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    args = parser.parse_args()
    command = [
        args.controller,
        "--ros-args",
        "-r", "__node:=test_depthless_phase_controller",
        "-p", "navigation_mode:=no_odom_phase",
        "-p", "acoustic_estimator_mode:=phase",
        "-p", "controller_profile:=real",
        "-p", "dry_run:=true",
        "-p", "imu_topic:=/test/depthless/imu",
        # No publisher is deliberately created for this topic.
        "-p", "depth_pose_topic:=/test/depthless/depth",
        "-p", "delta_range_topic:=/test/depthless/delta",
        "-p", "status_topic:=/test/depthless/status",
        "-p", "tank_max_depth_m:=2.0",
        "-p", "audio_timeout_s:=0.5",
        "-p", "imu_timeout_s:=0.5",
        "-p", "depth_pose_timeout_s:=0.5",
        "-p", "no_odom_horizontal_only:=true",
        "-p", "no_odom_vertical_control_enabled:=false",
        "-p", "no_odom_probe_leg_s:=0.4",
        "-p", "no_odom_probe_neutral_s:=0.2",
        "-p", "no_odom_probe_settle_s:=0.2",
        "-p", "no_odom_probe_sample_delay_s:=0.12",
    ]
    process = subprocess.Popen(command)
    rclpy.init()
    inputs = DepthlessPhaseInputs()
    try:
        deadline = time.monotonic() + 6.0
        saw_probe = False
        saw_xy_only = False
        while time.monotonic() < deadline:
            inputs.publish()
            rclpy.spin_once(inputs, timeout_sec=0.02)
            status = inputs.status
            if not status:
                continue
            no_odom = status.get("no_odom_phase", {})
            if status.get("state") == "NO_ODOM_PHASE_PROBE":
                saw_probe = True
            requested = status.get("requested_command", {})
            saw_xy_only |= (
                no_odom.get("horizontal_only") is True
                and abs(float(requested.get("heave", 0.0))) < 1.0e-9
                and status.get("depth_fresh") is False
            )
            if saw_probe and saw_xy_only:
                break
        if not saw_probe:
            raise AssertionError("depthless XY Phase controller remained in WAIT_VEHICLE")
        if not saw_xy_only:
            raise AssertionError("depthless XY Phase controller requested vertical control")
        print("no_odom_xy_depthless_runtime=PASS")
        return 0
    finally:
        inputs.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
