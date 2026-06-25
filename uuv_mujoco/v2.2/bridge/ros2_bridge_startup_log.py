"""Startup logging and optional-feature warnings for the ROS2 bridge."""

from __future__ import annotations

from .ros2_topic_registry import build_ros2_bridge_active_log


def log_ros2_bridge_startup(self) -> None:
    self.node.get_logger().info(
        build_ros2_bridge_active_log(mavros_surface_enabled=bool(self._mavros_surface_enabled))
    )
    if self._ping360_config.publish_echo and self.SonarEcho is None:
        self.node.get_logger().warn(
            "ping360_sonar_msgs/msg/SonarEcho is not installed; "
            "/ping360/scan_echo and /ping360/echo are disabled. "
            "Build/source rospkg/ping360_sonar_msgs or the upstream ping360_sonar workspace."
        )
    if self._legacy_image_request:
        self.node.get_logger().warn(
            "Legacy image publishing flags were requested, but the lightweight bridge "
            "does not publish /stereo/* topics."
        )


__all__ = ["log_ros2_bridge_startup"]
