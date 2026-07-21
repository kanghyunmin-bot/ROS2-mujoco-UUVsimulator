#!/usr/bin/env python3
"""Verify slow XY feedback extends a no-odometry Phase probe leg.

The velocity topic is deliberately used as timing feedback only. This test
keeps its XY speed at zero and asserts that the active ABBA leg is extended;
it does not need a source position or a successful acoustic bearing fit.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String


class SlowResponseInputs(Node):
    def __init__(self) -> None:
        super().__init__("motion_response_inputs")
        self.imu_pub = self.create_publisher(Imu, "/test/motion_response/imu", 20)
        self.delta_pub = self.create_publisher(Float64, "/test/motion_response/delta", 50)
        self.velocity_pub = self.create_publisher(Odometry, "/test/motion_response/velocity", 20)
        self.status: dict = {}
        self.create_subscription(String, "/test/motion_response/status", self._on_status, 20)

    def _on_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self.status = value

    def publish(self) -> None:
        imu = Imu()
        imu.orientation.w = 1.0
        self.imu_pub.publish(imu)
        self.delta_pub.publish(Float64(data=0.001))
        # Explicitly zero XY speed: the controller must extend the current
        # phase leg before moving on to the next ABBA direction.
        self.velocity_pub.publish(Odometry())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    args = parser.parse_args()
    command = [
        args.controller,
        "--ros-args",
        "-r", "__node:=test_motion_response_controller",
        "-p", "navigation_mode:=no_odom_phase",
        "-p", "acoustic_estimator_mode:=phase",
        "-p", "controller_profile:=real",
        "-p", "dry_run:=true",
        "-p", "imu_topic:=/test/motion_response/imu",
        "-p", "delta_range_topic:=/test/motion_response/delta",
        "-p", "status_topic:=/test/motion_response/status",
        "-p", "motion_response_enabled:=true",
        "-p", "motion_response_velocity_topic:=/test/motion_response/velocity",
        "-p", "motion_response_min_speed_mps:=0.03",
        "-p", "motion_response_probe_extension_s:=0.10",
        "-p", "motion_response_probe_max_extension_s:=0.20",
        "-p", "motion_response_feedback_timeout_s:=0.50",
        "-p", "no_odom_horizontal_only:=true",
        "-p", "no_odom_vertical_control_enabled:=false",
        "-p", "no_odom_probe_leg_s:=0.50",
        "-p", "no_odom_probe_neutral_s:=0.20",
        "-p", "no_odom_probe_settle_s:=0.20",
        "-p", "no_odom_probe_sample_delay_s:=0.12",
    ]
    process = subprocess.Popen(command)
    rclpy.init()
    inputs = SlowResponseInputs()
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            inputs.publish()
            rclpy.spin_once(inputs, timeout_sec=0.02)
            response = inputs.status.get("motion_response", {})
            if (
                inputs.status.get("state") == "NO_ODOM_PHASE_PROBE"
                and response.get("feedback_fresh") is True
                and float(response.get("probe_extension_s", 0.0)) > 0.0
            ):
                print("motion_response_runtime=PASS")
                return 0
        raise AssertionError(f"slow feedback did not extend a Phase leg: {inputs.status}")
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
