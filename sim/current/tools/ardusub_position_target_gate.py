#!/usr/bin/env python3
"""Gate ROS position targets for the ArduSub 4.1 SITL controller.

ArduSub 4.1 rebuilds its waypoint S-curve whenever it receives an absolute
position target. Streaming an unchanged target at the usual offboard rate can
therefore keep the trajectory at its initial point forever. This simulation
adapter forwards the first target immediately, suppresses exact repeats, and
rate-limits genuinely moving targets such as the region-scan look-ahead point.
"""

import math

import rclpy
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ArduSubPositionTargetGate(Node):
    def __init__(self) -> None:
        super().__init__("ardusub_position_target_gate")
        self.declare_parameter("input_topic", "/guided/setpoint_raw/local")
        self.declare_parameter("output_topic", "/mavros/setpoint_raw/local")
        self.declare_parameter("min_update_interval_s", 0.75)
        self.declare_parameter("position_epsilon_m", 0.02)
        self.declare_parameter("yaw_epsilon_rad", 0.0174532925)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.min_interval_s = max(
            0.1, float(self.get_parameter("min_update_interval_s").value)
        )
        self.position_epsilon_m = max(
            0.001, float(self.get_parameter("position_epsilon_m").value)
        )
        self.yaw_epsilon_rad = max(
            0.001, float(self.get_parameter("yaw_epsilon_rad").value)
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(PositionTarget, output_topic, qos)
        self.subscription = self.create_subscription(
            PositionTarget, input_topic, self.on_target, qos
        )
        self.pending: PositionTarget | None = None
        self.last_sent: PositionTarget | None = None
        self.last_sent_ns = 0
        self.timer = self.create_timer(0.05, self.flush_pending)
        self.get_logger().info(
            "ArduSub 4.1 position-target gate ready: "
            f"{input_topic} -> {output_topic}, "
            f"minimum moving-target interval={self.min_interval_s:.2f}s"
        )

    @staticmethod
    def wrapped_angle_difference(first: float, second: float) -> float:
        return abs(math.atan2(math.sin(first - second), math.cos(first - second)))

    def materially_changed(
        self, candidate: PositionTarget, reference: PositionTarget
    ) -> bool:
        if (
            candidate.coordinate_frame != reference.coordinate_frame
            or candidate.type_mask != reference.type_mask
        ):
            return True
        dx = candidate.position.x - reference.position.x
        dy = candidate.position.y - reference.position.y
        dz = candidate.position.z - reference.position.z
        if math.sqrt(dx * dx + dy * dy + dz * dz) >= self.position_epsilon_m:
            return True
        return (
            self.wrapped_angle_difference(candidate.yaw, reference.yaw)
            >= self.yaw_epsilon_rad
        )

    def on_target(self, message: PositionTarget) -> None:
        if self.last_sent is None:
            self.send(message)
            return
        reference = self.pending if self.pending is not None else self.last_sent
        if self.materially_changed(message, reference):
            self.pending = message

    def flush_pending(self) -> None:
        if self.pending is None:
            return
        elapsed_s = (self.get_clock().now().nanoseconds - self.last_sent_ns) * 1e-9
        if elapsed_s < self.min_interval_s:
            return
        message = self.pending
        self.pending = None
        self.send(message)

    def send(self, message: PositionTarget) -> None:
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)
        self.last_sent = message
        self.last_sent_ns = self.get_clock().now().nanoseconds


def main() -> None:
    rclpy.init()
    node = ArduSubPositionTargetGate()
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
