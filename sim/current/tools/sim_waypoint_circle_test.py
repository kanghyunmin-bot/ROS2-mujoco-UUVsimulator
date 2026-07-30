#!/usr/bin/env python3
"""Publish one measured circular waypoint trajectory for sim motion QA."""

import math
from typing import Optional

import rclpy
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CircleWaypointTest(Node):
    def __init__(self) -> None:
        super().__init__("sim_waypoint_circle_test")
        self.declare_parameter("odometry_topic", "/mavros/local_position/odom")
        self.declare_parameter("waypoint_topic", "/waypoint")
        self.declare_parameter("radius_m", 1.5)
        self.declare_parameter("angular_speed_rad_s", 0.15)
        self.declare_parameter("start_tolerance_m", 0.13)
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("final_hold_s", 3.0)

        self.radius_m = max(0.2, float(self.get_parameter("radius_m").value))
        self.angular_speed = max(
            0.03, float(self.get_parameter("angular_speed_rad_s").value)
        )
        self.start_tolerance_m = max(
            0.03, float(self.get_parameter("start_tolerance_m").value)
        )
        self.settle_s = max(0.0, float(self.get_parameter("settle_s").value))
        self.final_hold_s = max(
            0.0, float(self.get_parameter("final_hold_s").value)
        )

        sensor_qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(
            PositionTarget,
            str(self.get_parameter("waypoint_topic").value),
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            sensor_qos,
        )
        self.odometry: Optional[Odometry] = None
        self.center_x = 0.0
        self.center_y = 0.0
        self.target_z = 0.0
        self.stage = "WAIT_ODOMETRY"
        self.stage_started_ns = 0
        self.circle_started_ns = 0
        self.timer = self.create_timer(0.05, self.tick)

    def on_odometry(self, message: Odometry) -> None:
        self.odometry = message
        if self.stage == "WAIT_ODOMETRY":
            position = message.pose.pose.position
            self.center_x = position.x
            self.center_y = position.y
            self.target_z = position.z
            self.stage = "MOVE_TO_START"
            self.stage_started_ns = self.get_clock().now().nanoseconds
            self.get_logger().info(
                "Circle QA initialized: "
                f"center=({self.center_x:.3f},{self.center_y:.3f}) "
                f"radius={self.radius_m:.2f} m z={self.target_z:.3f}"
            )

    def publish_target(self, angle: float) -> None:
        message = PositionTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = (
            self.odometry.header.frame_id if self.odometry is not None else "map"
        )
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position.x = self.center_x + self.radius_m * math.cos(angle)
        message.position.y = self.center_y + self.radius_m * math.sin(angle)
        message.position.z = self.target_z
        self.publisher.publish(message)

    def tick(self) -> None:
        if self.odometry is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        position = self.odometry.pose.pose.position

        if self.stage == "MOVE_TO_START":
            self.publish_target(0.0)
            distance = math.hypot(
                position.x - (self.center_x + self.radius_m),
                position.y - self.center_y,
            )
            if distance <= self.start_tolerance_m:
                self.stage = "SETTLE"
                self.stage_started_ns = now_ns
                self.get_logger().info(
                    f"Circle start reached with error={distance:.3f} m"
                )
            return

        if self.stage == "SETTLE":
            self.publish_target(0.0)
            if (now_ns - self.stage_started_ns) * 1.0e-9 >= self.settle_s:
                self.stage = "CIRCLE"
                self.circle_started_ns = now_ns
                self.get_logger().info("Circle traversal started")
            return

        if self.stage == "CIRCLE":
            elapsed_s = (now_ns - self.circle_started_ns) * 1.0e-9
            angle = self.angular_speed * elapsed_s
            if angle >= 2.0 * math.pi:
                self.stage = "FINAL_HOLD"
                self.stage_started_ns = now_ns
                self.get_logger().info("Circle traversal completed: 360 deg")
                angle = 0.0
            self.publish_target(angle)
            return

        if self.stage == "FINAL_HOLD":
            self.publish_target(0.0)
            if (now_ns - self.stage_started_ns) * 1.0e-9 >= self.final_hold_s:
                self.stage = "DONE"
                self.get_logger().info("Circle QA done")
                rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = CircleWaypointTest()
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
