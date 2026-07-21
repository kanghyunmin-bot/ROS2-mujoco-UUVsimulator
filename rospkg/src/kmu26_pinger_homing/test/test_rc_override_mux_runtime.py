#!/usr/bin/env python3
"""Runtime ownership test for the RC override multiplexer."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time

os.environ.setdefault("ROS_DOMAIN_ID", "177")

import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("rc_override_mux_test_probe")
        self.mission_pub = self.create_publisher(
            OverrideRCIn, "/test/mux/mission", 10
        )
        self.vision_pub = self.create_publisher(
            OverrideRCIn, "/test/mux/vision", 10
        )
        # A silent second publisher models an already-running MAVROS RC owner.
        self.decoy_output_pub = self.create_publisher(
            OverrideRCIn, "/test/mux/output", 10
        )
        self.output: OverrideRCIn | None = None
        self.owner = ""
        self.output_sub = self.create_subscription(
            OverrideRCIn, "/test/mux/output", self._on_output, 10
        )
        self.status_sub = self.create_subscription(
            String, "/control/rc_override_mux/status", self._on_status, 10
        )

    def _on_output(self, message: OverrideRCIn) -> None:
        self.output = message

    def _on_status(self, message: String) -> None:
        try:
            self.owner = str(json.loads(message.data).get("owner", ""))
        except json.JSONDecodeError:
            self.owner = "invalid"

    @staticmethod
    def release_frame() -> OverrideRCIn:
        message = OverrideRCIn()
        message.channels = [OverrideRCIn.CHAN_RELEASE] * 18
        return message

    @staticmethod
    def active_frame(pwm: int) -> OverrideRCIn:
        message = OverrideRCIn()
        message.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        message.channels[4] = pwm
        return message


def wait_for(probe: Probe, predicate, timeout: float, publish) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        publish()
        rclpy.spin_once(probe, timeout_sec=0.02)
        if predicate():
            return
    channel = None if probe.output is None else probe.output.channels[4]
    raise RuntimeError(
        f"timed out waiting for mux contract; owner={probe.owner!r} channel5={channel}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mux", required=True)
    args = parser.parse_args()
    environment = os.environ.copy()
    command = [
        args.mux,
        "--ros-args",
        "-p", "output_topic:=/test/mux/output",
        "-p", "joystick_topic:=/test/mux/joystick",
        "-p", "pinger_topic:=/test/mux/pinger",
        "-p", "mission_topic:=/test/mux/mission",
        "-p", "vision_topic:=/test/mux/vision",
        "-p", "stale_timeout_s:=0.20",
        "-p", "rate_hz:=100.0",
        "-p", "require_exclusive_output:=true",
        "-p", "output_discovery_grace_s:=0.25",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=environment,
    )
    rclpy.init()
    probe = Probe()
    try:
        mission_release = probe.release_frame()
        vision_active = probe.active_frame(1600)

        def publish_handoff() -> None:
            probe.mission_pub.publish(mission_release)
            probe.vision_pub.publish(vision_active)

        wait_for(
            probe,
            lambda: probe.output is None and probe.owner == "conflict",
            2.0,
            publish_handoff,
        )
        probe.destroy_publisher(probe.decoy_output_pub)

        wait_for(
            probe,
            lambda: probe.output is not None
            and probe.output.channels[4] == 1600
            and probe.owner == "vision",
            3.0,
            publish_handoff,
        )

        mission_active = probe.active_frame(1550)

        def publish_priority() -> None:
            probe.mission_pub.publish(mission_active)
            probe.vision_pub.publish(vision_active)

        wait_for(
            probe,
            lambda: probe.output is not None
            and probe.output.channels[4] == 1550
            and probe.owner == "mission",
            1.0,
            publish_priority,
        )

        wait_for(
            probe,
            lambda: probe.output is not None
            and all(value == OverrideRCIn.CHAN_RELEASE for value in probe.output.channels)
            and probe.owner == "none",
            1.0,
            lambda: None,
        )
        print("rc_override_mux_runtime=PASS")
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
            raise RuntimeError(f"mux exited {process.returncode}:\n{output}")


if __name__ == "__main__":
    raise SystemExit(main())
