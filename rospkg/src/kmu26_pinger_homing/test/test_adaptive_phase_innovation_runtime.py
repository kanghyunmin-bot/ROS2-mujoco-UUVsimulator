#!/usr/bin/env python3
"""Exercise the no-odometry Phase innovation re-estimate gate.

The synthetic ABBA data points toward body-forward.  Once the controller has
aligned and starts its forward segment, the test deliberately publishes a
persistent *opening* Phase delta.  The controller must return neutrally to a
new ABBA probe because of the acoustic innovation, well before its long
approach watchdog.  No position/ground-truth input is created.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String


class InnovationInputs(Node):
    def __init__(self) -> None:
        super().__init__("adaptive_phase_innovation_inputs")
        self.imu_pub = self.create_publisher(Imu, "/test/innovation/imu", 20)
        self.delta_pub = self.create_publisher(Float64, "/test/innovation/delta", 50)
        self.status: dict = {}
        self.create_subscription(String, "/test/innovation/status", self._on_status, 20)

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
        phase = self.status.get("phase")
        no_odom = self.status.get("no_odom_phase", {})
        axis = int(no_odom.get("probe_axis", 0))
        if phase == "NO_ODOM_PHASE_PROBE":
            # ABBA score: source is forward (+X).  The estimator records
            # command y in FLU, so both lateral legs are zero here.
            delta = -0.003 if axis == 1 else (0.003 if axis == -1 else 0.0)
        elif phase == "APPROACH":
            # Contradict the forward bearing: range is consistently opening.
            delta = 0.003
        else:
            delta = 0.0
        self.delta_pub.publish(Float64(data=delta))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    args = parser.parse_args()
    command = [
        args.controller,
        "--ros-args",
        "-r", "__node:=test_adaptive_phase_innovation_controller",
        "-p", "navigation_mode:=no_odom_phase",
        "-p", "acoustic_estimator_mode:=phase",
        "-p", "controller_profile:=real",
        "-p", "dry_run:=true",
        "-p", "imu_topic:=/test/innovation/imu",
        "-p", "delta_range_topic:=/test/innovation/delta",
        "-p", "status_topic:=/test/innovation/status",
        "-p", "tank_max_depth_m:=0.0",
        "-p", "audio_timeout_s:=0.5",
        "-p", "imu_timeout_s:=0.5",
        "-p", "motion_response_enabled:=false",
        "-p", "no_odom_horizontal_only:=true",
        "-p", "no_odom_vertical_control_enabled:=false",
        "-p", "no_odom_probe_leg_s:=0.50",
        "-p", "no_odom_probe_neutral_s:=0.20",
        "-p", "no_odom_probe_settle_s:=0.20",
        "-p", "no_odom_probe_sample_delay_s:=0.10",
        "-p", "no_odom_min_samples_per_leg:=2",
        "-p", "no_odom_initial_confirmation_probes:=1",
        "-p", "no_odom_reestimate_policy:=adaptive",
        "-p", "no_odom_approach_min_s:=0.30",
        "-p", "no_odom_approach_max_s:=10.0",
        "-p", "no_odom_innovation_window_s:=0.25",
        "-p", "no_odom_innovation_noise_floor_m:=0.0001",
        "-p", "no_odom_innovation_limit:=0.50",
        "-p", "no_odom_innovation_hold_s:=0.25",
        "-p", "no_odom_innovation_min_expected_delta_m:=0.00001",
    ]
    process = subprocess.Popen(command)
    rclpy.init()
    inputs = InnovationInputs()
    try:
        deadline = time.monotonic() + 12.0
        saw_approach = False
        while time.monotonic() < deadline:
            inputs.publish()
            rclpy.spin_once(inputs, timeout_sec=0.02)
            no_odom = inputs.status.get("no_odom_phase", {})
            saw_approach |= inputs.status.get("state") == "APPROACH"
            if (
                saw_approach
                and inputs.status.get("state") == "NO_ODOM_PHASE_PROBE"
                and int(no_odom.get("innovation_reestimate_count", 0)) >= 1
            ):
                print("adaptive_phase_innovation_runtime=PASS")
                return 0
        raise AssertionError(f"Phase innovation did not trigger ABBA re-estimate: {inputs.status}")
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
