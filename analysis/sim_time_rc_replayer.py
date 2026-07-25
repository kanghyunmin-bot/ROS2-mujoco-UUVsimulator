#!/usr/bin/env python3
"""Replay /mavros/rc/override according to simulation /clock, not wall time."""

from __future__ import annotations

import argparse
import copy
import json
import time
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


def clock_ns(msg: Clock) -> int:
    return int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)


AXIS_CHANNEL_INDICES = (3, 4, 5, 2)  # yaw, forward, lateral, heave


def apply_axis_matrix(
    samples: list[tuple[int, OverrideRCIn]], matrix: np.ndarray
) -> list[tuple[int, OverrideRCIn]]:
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("axis correction matrix must be finite 4x4")
    last_channels = [1500] * 18
    transformed = []
    for timestamp, source in samples:
        message = copy.deepcopy(source)
        for index, value in enumerate(source.channels):
            if value not in (0, 65535):
                last_channels[index] = int(value)
        command = np.asarray(
            [(last_channels[index] - 1500.0) / 400.0 for index in AXIS_CHANNEL_INDICES]
        )
        corrected = np.clip(matrix @ command, -1.0, 1.0)
        for index, value in zip(AXIS_CHANNEL_INDICES, corrected):
            message.channels[index] = int(round(1500.0 + 400.0 * value))
        transformed.append((timestamp, message))
    return transformed


def load_axis_matrix(path: Path | None) -> np.ndarray | None:
    if path is None:
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    values = payload.get("matrix", payload) if isinstance(payload, dict) else payload
    return np.asarray(values, dtype=float)


def load_samples(
    bag: Path, axis_matrix: np.ndarray | None = None
) -> list[tuple[int, OverrideRCIn]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    samples = []
    first_timestamp = None
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if topic != "/mavros/rc/override":
            continue
        if first_timestamp is None:
            first_timestamp = timestamp
        samples.append((timestamp - first_timestamp, deserialize_message(raw, OverrideRCIn)))
    if not samples:
        raise RuntimeError("bag has no /mavros/rc/override samples")
    if axis_matrix is not None:
        samples = apply_axis_matrix(samples, axis_matrix)
    return samples


class SimTimeRcReplayer(Node):
    def __init__(self, samples: list[tuple[int, OverrideRCIn]]) -> None:
        super().__init__("sim_time_rc_replayer")
        self.samples = samples
        self.index = 0
        self.latest_clock_ns: int | None = None
        self.start_clock_ns: int | None = None
        self.finish_clock_ns: int | None = None
        self.active = False
        self.done = False
        self.start_wall: float | None = None
        self.finish_wall: float | None = None

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(OverrideRCIn, "/mavros/rc/override", command_qos)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_service(Trigger, "/sim_time_rc_replay/start", self.on_start)
        self.get_logger().info(
            f"ready: samples={len(samples)} sim_span={samples[-1][0] * 1.0e-9:.6f}s; "
            "call /sim_time_rc_replay/start"
        )

    def on_start(self, _request, response):
        if self.active or self.done:
            response.success = False
            response.message = "replay already started"
            return response
        if self.latest_clock_ns is None:
            response.success = False
            response.message = "no /clock sample received"
            return response
        if self.publisher.get_subscription_count() < 1:
            response.success = False
            response.message = "no /mavros/rc/override subscriber"
            return response
        self.start_clock_ns = self.latest_clock_ns
        self.start_wall = time.monotonic()
        self.active = True
        response.success = True
        response.message = f"started at sim_clock_ns={self.start_clock_ns}"
        self.get_logger().info(response.message)
        return response

    def on_clock(self, msg: Clock) -> None:
        self.latest_clock_ns = clock_ns(msg)
        if not self.active or self.start_clock_ns is None:
            return
        elapsed_ns = self.latest_clock_ns - self.start_clock_ns
        while self.index < len(self.samples) and self.samples[self.index][0] <= elapsed_ns:
            self.publisher.publish(self.samples[self.index][1])
            self.index += 1
        if self.index == len(self.samples) and self.finish_clock_ns is None:
            self.finish_clock_ns = self.latest_clock_ns
            self.finish_wall = time.monotonic()
            self.done = True
            self.active = False
            wall_s = self.finish_wall - self.start_wall
            sim_s = (self.finish_clock_ns - self.start_clock_ns) * 1.0e-9
            self.get_logger().info(
                f"complete: published={self.index} sim_elapsed={sim_s:.6f}s "
                f"wall_elapsed={wall_s:.6f}s measured_rtf={sim_s / wall_s:.6f}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--axis-matrix-json",
        type=Path,
        help="optional 4x4 yaw/forward/lateral/heave command correction matrix",
    )
    args = parser.parse_args()
    axis_matrix = load_axis_matrix(args.axis_matrix_json)
    samples = load_samples(args.bag, axis_matrix)
    rclpy.init()
    node = SimTimeRcReplayer(samples)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
