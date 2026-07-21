#!/usr/bin/env python3
"""Live-mode fail-closed contract with synthetic physical ROS inputs."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "183")

import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("pinger_homing_live_runtime_probe")
        self.odom_pub = self.create_publisher(Odometry, "/test/live/odom", 10)
        self.state_pub = self.create_publisher(State, "/test/live/state", 10)
        self.delta_pub = self.create_publisher(Float64, "/test/live/delta", 10)
        self.iq_pub = self.create_publisher(Float64, "/test/live/iq", 10)
        self.direction_pub = self.create_publisher(
            Vector3Stamped, "/test/live/direction", 10
        )
        self.status: dict = {}
        self.rc: OverrideRCIn | None = None
        self.create_subscription(String, "/test/live/status", self._on_status, 10)
        self.create_subscription(OverrideRCIn, "/test/live/rc", self._on_rc, 10)

    def _on_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self.status = value

    def _on_rc(self, message: OverrideRCIn) -> None:
        self.rc = message

    def publish_inputs(self, *, armed: bool, audio: bool = True) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.z = -0.4
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        state = State()
        state.connected = True
        state.armed = armed
        state.mode = "MANUAL"
        self.state_pub.publish(state)

        if audio:
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

    def release_received(self) -> bool:
        return self.rc is not None and all(
            value == OverrideRCIn.CHAN_RELEASE for value in self.rc.channels
        )

    def active_received(self) -> bool:
        return self.rc is not None and any(
            value not in (OverrideRCIn.CHAN_RELEASE, OverrideRCIn.CHAN_NOCHANGE)
            for value in self.rc.channels
        )


def wait_for(probe: Probe, timeout_s: float, publish, predicate) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        publish()
        rclpy.spin_once(probe, timeout_sec=0.03)
        if predicate():
            return
    raise RuntimeError(f"live safety condition timed out: status={probe.status}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    args = parser.parse_args()
    command = [
        sys.executable,
        args.controller,
        "--ros-args",
        "-p", "dry_run:=false",
        "-p", "odometry_topic:=/test/live/odom",
        "-p", "vehicle_state_topic:=/test/live/state",
        "-p", "delta_range_topic:=/test/live/delta",
        "-p", "iq_magnitude_topic:=/test/live/iq",
        "-p", "direction_input_topic:=/test/live/direction",
        "-p", "status_topic:=/test/live/status",
        "-p", "rc_output_topic:=/test/live/rc",
        "-p", "rate_hz:=40.0",
        "-p", "tank_max_depth_m:=1.5",
        "-p", "audio_timeout_s:=0.5",
        "-p", "arrival_radius_m:=0.0",
        "-p", "max_runtime_s:=10.0",
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
        wait_for(
            probe,
            3.0,
            lambda: probe.publish_inputs(armed=False),
            lambda: probe.release_received()
            and probe.status.get("state") == "WAIT_VEHICLE"
            and probe.status.get("control_output_active") is False,
        )
        wait_for(
            probe,
            3.0,
            lambda: probe.publish_inputs(armed=True),
            lambda: probe.active_received()
            and probe.status.get("state") == "PROBE"
            and probe.status.get("control_output_active") is True,
        )
        wait_for(
            probe,
            2.0,
            lambda: probe.publish_inputs(armed=True, audio=False),
            lambda: probe.release_received()
            and probe.status.get("state") == "WAIT_VEHICLE"
            and probe.status.get("audio_fresh") is False
            and probe.status.get("control_output_active") is False,
        )
        print("pinger_homing_live_runtime=PASS unarmed=RELEASE armed=ACTIVE stale=RELEASE")
        return 0
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
