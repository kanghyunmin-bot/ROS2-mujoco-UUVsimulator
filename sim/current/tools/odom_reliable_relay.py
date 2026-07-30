#!/usr/bin/env python3
"""Relay MAVROS BEST_EFFORT odometry as RELIABLE for legacy controllers."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ReliableOdometryRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_reliable_relay")
        self.declare_parameter("input_topic", "/mavros/local_position/odom")
        self.declare_parameter("output_topic", "/hydrophone/odometry")

        input_qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        output_qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(Odometry, output_topic, output_qos)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self.publisher.publish,
            input_qos,
        )
        self.get_logger().info(
            "Odometry QoS relay ready: BEST_EFFORT "
            f"{self.get_parameter('input_topic').value} -> RELIABLE {output_topic}"
        )


def main() -> None:
    rclpy.init()
    node = ReliableOdometryRelay()
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
