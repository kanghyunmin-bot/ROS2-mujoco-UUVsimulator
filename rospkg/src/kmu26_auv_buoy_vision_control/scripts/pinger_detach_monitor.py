#!/usr/bin/env python3
"""Convert live MuJoCo course-buoy status into a simulator-neutral Bool contract."""

from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class PingerDetachMonitor(Node):
    def __init__(self) -> None:
        super().__init__("pinger_detach_monitor")
        self.declare_parameter(
            "status_topic", "/mujoco/course_buoys/status"
        )
        self.declare_parameter(
            "detached_topic", "/vision/pinger_detached"
        )
        self.declare_parameter(
            "target_id", "course_buoy_pinger_white_1_float"
        )

        status_topic = str(self.get_parameter("status_topic").value)
        detached_topic = str(self.get_parameter("detached_topic").value)
        self.target_id = str(self.get_parameter("target_id").value).strip()
        if not self.target_id:
            raise ValueError("target_id must not be empty")

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.detached_pub = self.create_publisher(Bool, detached_topic, latched_qos)
        self.create_subscription(String, status_topic, self._on_status, 10)
        self._last_detached: bool | None = None
        self._publish(False)
        self.get_logger().info(
            f"Monitoring physical target={self.target_id} on {status_topic}; "
            f"publishing {detached_topic}"
        )

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"Ignoring invalid course-buoy status JSON: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        rows = payload.get("buoys", [])
        if not isinstance(rows, list):
            return
        for row in rows:
            if not isinstance(row, dict) or str(row.get("id", "")) != self.target_id:
                continue
            # A free surface float has detached=true but no magnet.  The
            # underwater pinger succeeds only after its real magnet releases.
            detached = bool(row.get("has_magnet", False)) and bool(
                row.get("detached", False)
            )
            self._publish(detached)
            return

    def _publish(self, detached: bool) -> None:
        if detached == self._last_detached:
            return
        self._last_detached = detached
        msg = Bool()
        msg.data = detached
        self.detached_pub.publish(msg)
        if detached:
            self.get_logger().info(
                f"Physical detach latched target={self.target_id}"
            )


def main() -> int:
    rclpy.init()
    node = PingerDetachMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
