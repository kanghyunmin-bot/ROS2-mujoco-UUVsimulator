#!/usr/bin/env python3
"""Check live ROS topic liveness for the MuJoCo + ArduSub runtime."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    label: str
    required: bool = True
    min_count: int = 1


@dataclass
class TopicStats:
    topic: str
    label: str
    required: bool
    min_count: int
    count: int = 0
    first_wall_s: float | None = None
    last_wall_s: float | None = None

    def record(self, wall_s: float) -> None:
        self.count += 1
        if self.first_wall_s is None:
            self.first_wall_s = wall_s
        self.last_wall_s = wall_s

    def rate_hz(self) -> float:
        if self.count <= 1 or self.first_wall_s is None or self.last_wall_s is None:
            return 0.0
        duration = max(1.0e-9, self.last_wall_s - self.first_wall_s)
        return float(self.count - 1) / duration

    def ok(self) -> bool:
        return (not self.required) or self.count >= self.min_count

    def payload(self) -> dict[str, object]:
        data = asdict(self)
        data["rate_hz"] = round(self.rate_hz(), 3)
        data["ok"] = self.ok()
        return data


TOPICS: tuple[TopicSpec, ...] = (
    TopicSpec("/mavros/state", "mavros_state"),
    TopicSpec("/mavros/rc/in", "rc_in", required=False, min_count=0),
    TopicSpec("/mavros/rc/out", "rc_out"),
    TopicSpec("/imu/data", "imu"),
    TopicSpec("/mavros/imu/data_raw", "mavros_imu_raw"),
    TopicSpec("/mavros/imu/static_pressure", "bar30_static_pressure"),
    TopicSpec("/bar30/pressure_pa", "bar30_pressure_pa"),
    TopicSpec("/depth", "depth"),
    TopicSpec("/dvl/velocity", "dvl_velocity"),
    TopicSpec("/dvl/altitude", "dvl_altitude"),
    TopicSpec("/mavros/local_position/odom", "local_odom"),
    TopicSpec("/ping360/status", "ping360_status"),
    TopicSpec("/ping360/scan", "ping360_scan"),
    TopicSpec("/ping360/image", "ping360_image"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=6.0)
    parser.add_argument("--out", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import RCIn, RCOut, State
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import FluidPressure, Image, Imu, LaserScan, Range
    from std_msgs.msg import Float32, String

    type_by_topic = {
        "/mavros/state": State,
        "/mavros/rc/in": RCIn,
        "/mavros/rc/out": RCOut,
        "/imu/data": Imu,
        "/mavros/imu/data_raw": Imu,
        "/mavros/imu/static_pressure": FluidPressure,
        "/bar30/pressure_pa": Float32,
        "/depth": Float32,
        "/dvl/velocity": TwistStamped,
        "/dvl/altitude": Range,
        "/mavros/local_position/odom": Odometry,
        "/ping360/status": String,
        "/ping360/scan": LaserScan,
        "/ping360/image": Image,
    }

    stats = {
        spec.topic: TopicStats(
            topic=spec.topic,
            label=spec.label,
            required=spec.required,
            min_count=spec.min_count,
        )
        for spec in TOPICS
    }

    class LivenessNode(Node):
        def __init__(self) -> None:
            super().__init__("uuv_live_topic_liveness_check")
            self._subs = []
            for spec in TOPICS:
                msg_type = type_by_topic[spec.topic]
                self._subs.append(self.create_subscription(msg_type, spec.topic, self._callback(spec.topic), 10))

        def _callback(self, topic: str):
            def cb(_msg) -> None:
                stats[topic].record(time.monotonic())

            return cb

    rclpy.init()
    node = LivenessNode()
    start = time.monotonic()
    try:
        while time.monotonic() - start < float(args.duration_s):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result = {
        "duration_s": float(args.duration_s),
        "topics": [stats[spec.topic].payload() for spec in TOPICS],
    }
    result["overall"] = "pass" if all(item["ok"] for item in result["topics"]) else "fail"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")

    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["overall"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
