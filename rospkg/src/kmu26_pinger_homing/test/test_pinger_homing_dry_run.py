#!/usr/bin/env python3
"""Runtime safety contract for the standalone pinger homing controller."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "181")

import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("pinger_homing_dry_run_probe")
        self.odom_pub = self.create_publisher(Odometry, "/test/pinger/odom", 10)
        self.state_pub = self.create_publisher(State, "/test/pinger/state", 10)
        self.delta_pub = self.create_publisher(Float64, "/test/pinger/delta", 10)
        self.iq_pub = self.create_publisher(Float64, "/test/pinger/iq", 10)
        self.direction_pub = self.create_publisher(
            Vector3Stamped, "/test/pinger/direction", 10
        )
        self.status: dict = {}
        self.rc: OverrideRCIn | None = None
        self.create_subscription(String, "/test/pinger/status", self._on_status, 10)
        self.create_subscription(OverrideRCIn, "/test/pinger/rc", self._on_rc, 10)

    def _on_status(self, message: String) -> None:
        try:
            parsed = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.status = parsed

    def _on_rc(self, message: OverrideRCIn) -> None:
        self.rc = message

    def publish_inputs(self) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.z = -0.45
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        state = State()
        state.connected = True
        state.armed = False
        self.state_pub.publish(state)

        delta = Float64()
        delta.data = -0.002
        self.delta_pub.publish(delta)

        iq = Float64()
        iq.data = 0.12
        self.iq_pub.publish(iq)

        direction = Vector3Stamped()
        direction.header.stamp = self.get_clock().now().to_msg()
        direction.vector.x = 1.0
        direction.vector.y = 0.1
        direction.vector.z = -0.2
        self.direction_pub.publish(direction)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    args = parser.parse_args()
    command = [
        sys.executable,
        args.controller,
        "--ros-args",
        "-p", "dry_run:=true",
        "-p", "odometry_topic:=/test/pinger/odom",
        "-p", "vehicle_state_topic:=/test/pinger/state",
        "-p", "delta_range_topic:=/test/pinger/delta",
        "-p", "iq_magnitude_topic:=/test/pinger/iq",
        "-p", "direction_input_topic:=/test/pinger/direction",
        "-p", "direction_output_topic:=/test/pinger/direction_body",
        "-p", "status_topic:=/test/pinger/status",
        "-p", "rc_output_topic:=/test/pinger/rc",
        "-p", "rate_hz:=40.0",
        "-p", "tank_max_depth_m:=1.5",
        "-p", "amplitude_range_constant:=0.0",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    rclpy.init()
    probe = Probe()
    try:
        deadline = time.monotonic() + 6.0
        saw_requested_motion = False
        while time.monotonic() < deadline:
            probe.publish_inputs()
            rclpy.spin_once(probe, timeout_sec=0.03)
            requested = probe.status.get("requested_command", {})
            if any(abs(float(requested.get(axis, 0.0))) > 1.0e-6 for axis in ("forward", "lateral", "heave", "yaw")):
                saw_requested_motion = True
            safe_rc = probe.rc is not None and all(
                value == OverrideRCIn.CHAN_RELEASE for value in probe.rc.channels
            )
            if (
                probe.status.get("state") == "PROBE"
                and probe.status.get("dry_run") is True
                and probe.status.get("control_output_active") is False
                and probe.status.get("raw_armed") is False
                and safe_rc
                and saw_requested_motion
            ):
                print("pinger_homing_dry_run=PASS state=PROBE rc=RELEASE")
                return 0
        raise RuntimeError(
            "dry-run safety contract timed out: "
            f"status={probe.status} rc={None if probe.rc is None else probe.rc.channels}"
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=1.0)
        if process.returncode not in (0, -15):
            raise RuntimeError(f"controller exited {process.returncode}:\n{output}")


if __name__ == "__main__":
    raise SystemExit(main())
