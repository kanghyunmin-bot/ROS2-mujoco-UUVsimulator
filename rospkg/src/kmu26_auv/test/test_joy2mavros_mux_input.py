#!/usr/bin/env python3
"""Verify joystick idle release and active takeover for the shared RC mux."""

from __future__ import annotations

import argparse
import os
import subprocess
import time

os.environ.setdefault("ROS_DOMAIN_ID", "185")

import rclpy  # noqa: E402
from mavros_msgs.msg import OverrideRCIn  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("joy2mavros_mux_input_probe")
        self.publisher = self.create_publisher(Joy, "/joy", 10)
        self.last: OverrideRCIn | None = None
        self.create_subscription(OverrideRCIn, "/test/joy/rc", self._on_rc, 10)

    def _on_rc(self, message: OverrideRCIn) -> None:
        self.last = message

    def publish(self, axes: dict[int, float] | None = None) -> None:
        message = Joy()
        message.axes = [0.0] * 8
        message.buttons = [0] * 12
        for index, value in (axes or {}).items():
            message.axes[index] = value
        self.publisher.publish(message)


def is_released(probe: Probe) -> bool:
    return probe.last is not None and all(
        value == OverrideRCIn.CHAN_RELEASE for value in probe.last.channels
    )


def wait_for(
    probe: Probe,
    axes: dict[int, float],
    predicate,
    timeout_s: float = 2.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        probe.publish(axes)
        rclpy.spin_once(probe, timeout_sec=0.03)
        if predicate():
            return
    raise RuntimeError("joy2mavros mux-input condition timed out")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    args = parser.parse_args()
    process = subprocess.Popen(
        [
            args.executable,
            "--ros-args",
            "-p", "rc_output_topic:=/test/joy/rc",
            "-p", "release_when_idle:=true",
            "-p", "axis_deadzone:=0.08",
            "-p", "vertical_axis_deadzone:=0.10",
            "-p", "alt_hold_entry_neutral_sec:=0.15",
            "-p", "alt_hold_post_entry_neutral_sec:=0.15",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    rclpy.init()
    probe = Probe()
    try:
        # The first Joy sample is intentionally consumed as the edge baseline.
        wait_for(probe, {}, lambda: is_released(probe))

        # Noise inside the configured deadzone must not take mux ownership.
        wait_for(probe, {1: 0.05}, lambda: is_released(probe))

        # Deliberate forward and vertical input must publish active PWM.
        wait_for(
            probe,
            {1: 0.4},
            lambda: probe.last is not None and probe.last.channels[4] > 1500,
        )
        wait_for(
            probe,
            {3: 0.3},
            lambda: probe.last is not None and probe.last.channels[2] > 1500,
        )
        wait_for(probe, {}, lambda: is_released(probe))

        # ALT_HOLD entry is an intentional temporary vertical-neutral override.
        wait_for(
            probe,
            {6: 1.0},
            lambda: probe.last is not None and probe.last.channels[2] == 1500,
        )
        wait_for(probe, {}, lambda: is_released(probe), timeout_s=2.5)

        print(
            "joy2mavros_mux_input=PASS "
            "idle=RELEASE deadzone=RELEASE motion=ACTIVE alt_hold=NEUTRAL"
        )
        return 0
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=1.0)
        if process.returncode not in (0, -15):
            raise RuntimeError(f"joy2mavros exited {process.returncode}:\n{output}")


if __name__ == "__main__":
    raise SystemExit(main())
