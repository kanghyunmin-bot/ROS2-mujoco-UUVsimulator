#!/usr/bin/env python3
"""Bridge MAVROS best-effort odometry to reliable QoS for SNR homing."""

from __future__ import annotations

import argparse

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class ReliableOdometryRelay(Node):
    def __init__(
        self, *, input_topic: str, output_topic: str, frame_id: str
    ) -> None:
        super().__init__("uuv_reliable_odometry_relay")
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
        )
        self._frame_id = frame_id
        self._publisher = self.create_publisher(Odometry, output_topic, reliable_qos)
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._on_odometry, best_effort_qos
        )
        self.get_logger().info(
            f"Relaying {input_topic} -> {output_topic} with reliable QoS "
            f"and frame_id={frame_id}"
        )

    def _on_odometry(self, message: Odometry) -> None:
        message.header.frame_id = self._frame_id
        self._publisher.publish(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/mavros/local_position/odom")
    parser.add_argument("--output-topic", default="/homing/vehicle_odom")
    parser.add_argument("--frame-id", default="odom")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ReliableOdometryRelay(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        frame_id=args.frame_id,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
