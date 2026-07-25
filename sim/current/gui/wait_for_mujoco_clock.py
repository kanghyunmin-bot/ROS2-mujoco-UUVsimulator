#!/usr/bin/env python3
"""Block simulator consumers until the private MuJoCo clock is usable."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def clock_seconds(message: Clock) -> float:
    return float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9


class ClockBarrier(Node):
    def __init__(self, topic: str, min_time_s: float, stable_samples: int) -> None:
        super().__init__("uuv_mujoco_clock_barrier")
        self._min_time_s = min_time_s
        self._required_samples = stable_samples
        self._last_time_s: float | None = None
        self._stable_samples = 0
        self.ready = False
        qos = QoSProfile(depth=max(10, stable_samples), reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, topic, self._on_clock, qos)

    def _on_clock(self, message: Clock) -> None:
        current = clock_seconds(message)
        if self._last_time_s is None or current + 1.0e-9 < self._last_time_s:
            self._stable_samples = 0
        self._last_time_s = current
        if current < self._min_time_s:
            self._stable_samples = 0
            return
        self._stable_samples += 1
        self.ready = self._stable_samples >= self._required_samples


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/uuv_mujoco/clock")
    parser.add_argument("--min-time", type=float, default=0.5)
    parser.add_argument("--stable-samples", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    node = ClockBarrier(
        args.topic,
        max(0.0, args.min_time),
        max(1, args.stable_samples),
    )
    deadline = time.monotonic() + max(1.0, args.timeout)
    try:
        while rclpy.ok() and not node.ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        ready = node.ready
        last_time_s = node._last_time_s
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not ready:
        print(
            f"MuJoCo clock barrier timed out: topic={args.topic} "
            f"last_time={last_time_s}",
            flush=True,
        )
        return 1
    print(
        f"MuJoCo clock ready: topic={args.topic} time={last_time_s:.6f}s",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
