"""Startup logging and optional-feature warnings for the ROS2 bridge."""

from __future__ import annotations

from .ros2_topic_registry import build_ros2_bridge_active_log
from .ros2_stereo_image import REAL_CAMERA_COMPRESSED_TOPIC
from .ros2_stereo_image import REAL_CAMERA_RAW_TOPIC


def log_ros2_bridge_startup(self) -> None:
    self.node.get_logger().info(
        build_ros2_bridge_active_log(mavros_surface_enabled=bool(self._mavros_surface_enabled))
    )
    if self._ping360_config.publish_echo and self.SonarEcho is None:
        self.node.get_logger().warn(
            "ping360_sonar_msgs/msg/SonarEcho is not installed; "
            "/ping360/scan_echo and /ping360/echo are disabled. "
            "Build/source rospkg/src/ping360_sonar_msgs or the upstream ping360_sonar workspace."
        )
    if getattr(self, "_stereo_image_enabled", False):
        self.node.get_logger().info(
            "Stereo image bridge active: /stereo/left/image_raw, /stereo/right/image_raw, "
            f"{REAL_CAMERA_RAW_TOPIC}, {REAL_CAMERA_COMPRESSED_TOPIC} "
            f"({self._stereo_image_width}x{self._stereo_image_height}@{self._stereo_image_hz:.1f}Hz on demand)."
        )


__all__ = ["log_ros2_bridge_startup"]
