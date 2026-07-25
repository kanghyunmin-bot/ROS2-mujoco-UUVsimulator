#!/usr/bin/env python3
"""Convert the hydrophone estimator's odom-frame direction into body FLU."""

from __future__ import annotations

import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data


def world_vector_to_body(
    vector: Iterable[float], quaternion_xyzw: Iterable[float]
) -> tuple[float, float, float]:
    """Apply the inverse body-to-world quaternion rotation to a vector."""

    vx, vy, vz = (float(value) for value in vector)
    qx, qy, qz, qw = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1.0e-9:
        raise ValueError("invalid zero-length orientation quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    # Columns of R are body axes expressed in world coordinates. R^T maps a
    # world vector into ROS body FLU without requiring a TF lookup.
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return (
        r00 * vx + r10 * vy + r20 * vz,
        r01 * vx + r11 * vy + r21 * vz,
        r02 * vx + r12 * vy + r22 * vz,
    )


class HomingDirectionBodyAdapter(Node):
    def __init__(self) -> None:
        super().__init__("homing_direction_body_adapter")
        self.declare_parameter("input_topic", "/homing/direction_world")
        self.declare_parameter("output_topic", "/homing/direction")
        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("odometry_timeout_s", 0.5)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        odometry_topic = str(self.get_parameter("odometry_topic").value)
        self._odom_timeout_s = float(self.get_parameter("odometry_timeout_s").value)
        self._orientation: tuple[float, float, float, float] | None = None
        self._odom_time = None

        self._publisher = self.create_publisher(Vector3Stamped, output_topic, 10)
        self.create_subscription(Odometry, odometry_topic, self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, input_topic, self._on_direction, 10)
        self.get_logger().info(
            f"homing direction adapter: {input_topic} (odom/world) -> {output_topic} (base_link FLU)"
        )

    def _on_odometry(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._orientation = (q.x, q.y, q.z, q.w)
        self._odom_time = self.get_clock().now()

    def _on_direction(self, msg: Vector3Stamped) -> None:
        if self._orientation is None or self._odom_time is None:
            return
        if (self.get_clock().now() - self._odom_time).nanoseconds * 1.0e-9 > self._odom_timeout_s:
            return
        try:
            body = world_vector_to_body(
                (msg.vector.x, msg.vector.y, msg.vector.z), self._orientation
            )
        except ValueError:
            return
        norm = math.sqrt(sum(value * value for value in body))
        if norm < 1.0e-9 or not all(math.isfinite(value) for value in body):
            return
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.vector.x, out.vector.y, out.vector.z = (value / norm for value in body)
        self._publisher.publish(out)


def main() -> int:
    rclpy.init()
    node = HomingDirectionBodyAdapter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
