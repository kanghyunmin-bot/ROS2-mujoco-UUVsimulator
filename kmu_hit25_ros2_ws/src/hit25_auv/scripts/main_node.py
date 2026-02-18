#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hit25_auv.controller import PositionController
from hit25_auv.map_odom_broadcaster import MapOdomBroadcaster


class Hit25AuvMainNode(Node):
    def __init__(self):
        super().__init__("hit25_auv_main")

        enable_map_odom = self.declare_parameter("enable_map_odom_broadcaster", True).value
        enable_controller = self.declare_parameter("enable_controller", True).value
        log_level = self.declare_parameter("log_level", "INFO").value

        self.get_logger().info("[hit25_auv_main] node started.")
        self.get_logger().info(f"  - enable_map_odom_broadcaster: {enable_map_odom}")
        self.get_logger().info(f"  - enable_controller: {enable_controller}")
        self.get_logger().info(f"  - (Configured log_level: {log_level})")

        self.map_odom = None
        self.controller = None

        if enable_map_odom:
            try:
                self.map_odom = MapOdomBroadcaster(self)
                self.get_logger().info("MapOdomBroadcaster created.")
            except Exception as e:
                self.get_logger().fatal(f"Failed to create MapOdomBroadcaster: {e}")
                raise
        else:
            self.get_logger().info("MapOdomBroadcaster is disabled by parameter.")

        if enable_controller:
            try:
                self.controller = PositionController(self)
                self.get_logger().info("PositionController created.")
            except Exception as e:
                self.get_logger().fatal(f"Failed to create PositionController: {e}")
                raise
        else:
            self.get_logger().info("PositionController is disabled by parameter.")

        if not enable_map_odom and not enable_controller:
            self.get_logger().warn(
                "No components enabled; node will spin but do nothing."
            )


def main():
    rclpy.init()
    node = None
    try:
        node = Hit25AuvMainNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
