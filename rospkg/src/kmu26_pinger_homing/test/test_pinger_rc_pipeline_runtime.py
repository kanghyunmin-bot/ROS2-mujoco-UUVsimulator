#!/usr/bin/env python3
"""End-to-end controller -> RC mux fail-closed runtime contract."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "185")

import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


class PipelineProbe(Node):
    def __init__(self) -> None:
        super().__init__("pinger_rc_pipeline_runtime_probe")
        self.odom_pub = self.create_publisher(Odometry, "/test/pipeline/odom", 10)
        self.state_pub = self.create_publisher(State, "/test/pipeline/state", 10)
        self.delta_pub = self.create_publisher(Float64, "/test/pipeline/delta", 10)
        self.iq_pub = self.create_publisher(Float64, "/test/pipeline/iq", 10)
        self.direction_pub = self.create_publisher(
            Vector3Stamped, "/test/pipeline/direction", 10
        )
        self.output: OverrideRCIn | None = None
        self.create_subscription(
            OverrideRCIn, "/test/pipeline/output", self._on_output, 10
        )

    def _on_output(self, message: OverrideRCIn) -> None:
        self.output = message

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

    def released(self) -> bool:
        return self.output is not None and all(
            value == OverrideRCIn.CHAN_RELEASE for value in self.output.channels
        )

    def active(self) -> bool:
        return self.output is not None and any(
            value not in (OverrideRCIn.CHAN_RELEASE, OverrideRCIn.CHAN_NOCHANGE)
            for value in self.output.channels
        )


def wait_for(probe: PipelineProbe, timeout_s: float, publish, predicate) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        publish()
        rclpy.spin_once(probe, timeout_sec=0.03)
        if predicate():
            return
    channels = None if probe.output is None else list(probe.output.channels)
    raise RuntimeError(f"pinger RC pipeline timed out: channels={channels}")


def stop_process(process: subprocess.Popen[str]) -> str:
    process.send_signal(signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            output, _ = process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=1.0)
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    parser.add_argument("--mux", required=True)
    args = parser.parse_args()

    controller = subprocess.Popen(
        [
            sys.executable,
            args.controller,
            "--ros-args",
            "-p", "dry_run:=false",
            "-p", "odometry_topic:=/test/pipeline/odom",
            "-p", "vehicle_state_topic:=/test/pipeline/state",
            "-p", "delta_range_topic:=/test/pipeline/delta",
            "-p", "iq_magnitude_topic:=/test/pipeline/iq",
            "-p", "direction_input_topic:=/test/pipeline/direction",
            "-p", "rc_output_topic:=/test/pipeline/pinger",
            "-p", "status_topic:=/test/pipeline/status",
            "-p", "rate_hz:=40.0",
            "-p", "tank_max_depth_m:=1.5",
            "-p", "audio_timeout_s:=0.5",
            "-p", "arrival_radius_m:=0.0",
            "-p", "max_runtime_s:=10.0",
            "-p", "amplitude_range_constant:=0.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    mux = subprocess.Popen(
        [
            args.mux,
            "--ros-args",
            "-p", "output_topic:=/test/pipeline/output",
            "-p", "pinger_topic:=/test/pipeline/pinger",
            "-p", "joystick_topic:=/test/pipeline/joystick",
            "-p", "mission_topic:=/test/pipeline/disabled_mission",
            "-p", "vision_topic:=/test/pipeline/disabled_vision",
            "-p", "stale_timeout_s:=0.25",
            "-p", "rate_hz:=100.0",
            "-p", "require_exclusive_output:=true",
            "-p", "output_discovery_grace_s:=0.25",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )

    rclpy.init()
    probe = PipelineProbe()
    try:
        wait_for(
            probe,
            3.0,
            lambda: probe.publish_inputs(armed=False),
            probe.released,
        )
        wait_for(
            probe,
            3.0,
            lambda: probe.publish_inputs(armed=True),
            probe.active,
        )
        wait_for(
            probe,
            2.0,
            lambda: probe.publish_inputs(armed=True, audio=False),
            probe.released,
        )
        print("pinger_rc_pipeline_runtime=PASS unarmed=RELEASE armed=ACTIVE stale=RELEASE")
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        controller_output = stop_process(controller)
        mux_output = stop_process(mux)
        for name, process, output in (
            ("controller", controller, controller_output),
            ("mux", mux, mux_output),
        ):
            if process.returncode not in (0, -2, -15):
                raise RuntimeError(f"{name} exited {process.returncode}:\n{output}")


if __name__ == "__main__":
    raise SystemExit(main())
