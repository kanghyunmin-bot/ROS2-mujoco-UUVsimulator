#!/usr/bin/env python3
"""Exercise the production C++ controller's filtered-odometry path.

This is deliberately a dry-run: it proves that fresh odometry and Phase
updates take the canonical controller from WAIT_VEHICLE into the legacy probe
sequence without requiring MAVROS arm or touching a vehicle.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time

os.environ.setdefault("ROS_DOMAIN_ID", "194")

import rclpy
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


class OdomRuntimeProbe(Node):
    def __init__(self, quality_only: bool) -> None:
        super().__init__("cpp_odometry_mode_runtime_probe")
        self.quality_only = quality_only
        self.odom_pub = self.create_publisher(Odometry, "/test/pinger/odometry", 20)
        self.delta_pub = self.create_publisher(Float64, "/test/pinger/delta", 50)
        self.iq_pub = self.create_publisher(Float64, "/test/pinger/iq", 20)
        self.quality_pub = self.create_publisher(Float64, "/test/pinger/quality", 20)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(State, "/test/pinger/state", state_qos)
        self.active_rc_seen = False
        self.create_subscription(OverrideRCIn, "/test/pinger/rc", self._on_rc, 20)
        self.status: dict = {}
        self.create_subscription(String, "/test/pinger/status", self._on_status, 20)
        self.x = 0.0

    def _on_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self.status = value

    def _on_rc(self, message: OverrideRCIn) -> None:
        inactive = {OverrideRCIn.CHAN_RELEASE, OverrideRCIn.CHAN_NOCHANGE, 1500}
        if any(int(value) not in inactive for value in message.channels):
            self.active_rc_seen = True

    def publish_inputs(self) -> None:
        # Slowly varying XYZ coordinates mirror localization output during a
        # real excitation and ensure Phase samples are paired with pose.
        self.x += 0.002
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = 0.10 * self.x
        odom.pose.pose.position.z = -0.7
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.06
        self.odom_pub.publish(odom)
        if not self.quality_only:
            self.delta_pub.publish(Float64(data=-0.0002))
        self.iq_pub.publish(Float64(data=1.0))
        self.quality_pub.publish(Float64(data=0.4))
        state = State()
        state.connected = True
        state.armed = True
        state.mode = "ALT_HOLD"
        self.state_pub.publish(state)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    parser.add_argument("--quality-only", action="store_true")
    args = parser.parse_args()
    process = subprocess.Popen([
        args.controller,
        "--ros-args",
        "-p", f"dry_run:={'false' if args.quality_only else 'true'}",
        "-p", "navigation_mode:=odometry",
        "-p", "legacy_python_sequence:=true",
        "-p", "mode:=ALT_HOLD",
        "-p", "rc_pwm_span:=400.0",
        "-p", "probe_pwm_delta:=20",
        "-p", "approach_pwm_delta:=25",
        "-p", "odometry_topic:=/test/pinger/odometry",
        "-p", "delta_range_topic:=/test/pinger/delta",
        "-p", "iq_magnitude_topic:=/test/pinger/iq",
        "-p", "audio_quality_topic:=/test/pinger/quality",
        "-p", "bootstrap_probe_on_audio_quality:=true",
        "-p", "status_topic:=/test/pinger/status",
        "-p", "rc_output_topic:=/test/pinger/rc",
        "-p", "direction_input_topic:=/test/pinger/direction",
        "-p", "imu_topic:=/test/pinger/imu",
        "-p", "depth_pose_topic:=/test/pinger/depth",
        "-p", "vehicle_state_topic:=/test/pinger/state",
        "-p", "max_runtime_s:=0.0",
    ])
    rclpy.init()
    probe = OdomRuntimeProbe(args.quality_only)
    try:
        deadline = time.monotonic() + 5.0
        accepted = False
        while time.monotonic() < deadline:
            probe.publish_inputs()
            rclpy.spin_once(probe, timeout_sec=0.04)
            status = probe.status
            if (
                status.get("navigation_mode") == "odometry"
                and status.get("odometry_required") is True
                and status.get("odometry_fresh") is True
                and status.get("audio_fresh") is True
                and status.get("legacy_python_sequence") is True
                and int(status.get("legacy_probe_pwm_delta", 0)) == 20
                and int(status.get("legacy_approach_pwm_delta", 0)) == 25
                and status.get("state") in {"PROBE", "REPROBE"}
                and (
                    (
                        args.quality_only
                        and int(status.get("sample_count", -1)) == 0
                        and status.get("phase_measurement_fresh") is False
                        and status.get("audio_quality_fresh") is True
                        and status.get("audio_bootstrap_active") is True
                        and status.get("control_output_active") is True
                        and probe.active_rc_seen
                    )
                    or (
                        not args.quality_only
                        and int(status.get("sample_count", 0)) > 0
                        and status.get("phase_measurement_fresh") is True
                    )
                )
            ):
                accepted = True
                break
        if not accepted:
            raise AssertionError(f"C++ odometry Phase path did not become ready: {probe.status}")
        mode = "quality-bootstrap" if args.quality_only else "phase-delta"
        print(f"cpp_odometry_mode_runtime=PASS state=PROBE source={mode}")
        return 0
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
