"""Startup logging and optional-feature warnings for the ROS2 bridge."""

from __future__ import annotations

from .ros2_topic_registry import build_ros2_bridge_active_log
from .ros2_stereo_image import (
    IMX219_CAMERA0_COMPRESSED_TOPIC,
    IMX219_CAMERA0_INFO_TOPIC,
    IMX219_CAMERA0_RAW_TOPIC,
    IMX219_CAMERA1_COMPRESSED_TOPIC,
    IMX219_CAMERA1_INFO_TOPIC,
    IMX219_CAMERA1_RAW_TOPIC,
    REAL_CAMERA_COMPRESSED_TOPIC,
    REAL_CAMERA_INFO_TOPIC,
    REAL_CAMERA_RAW_TOPIC,
)
from sim.contracts.ground_truth import (
    UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING,
)


def log_ros2_bridge_startup(self) -> None:
    self.node.get_logger().info(
        build_ros2_bridge_active_log(
            mavros_surface_enabled=bool(self._mavros_surface_enabled),
            strict_sitl_sensor_transport=bool(
                getattr(self, "_strict_sitl_sensor_transport", False)
            ),
            unsafe_legacy_ground_truth_odometry_filtered=bool(
                getattr(
                    self,
                    "_unsafe_legacy_ground_truth_odometry_filtered",
                    False,
                )
            ),
        )
    )
    if bool(
        getattr(
            self,
            "_unsafe_legacy_ground_truth_odometry_filtered",
            False,
        )
    ):
        self.node.get_logger().error(
            UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING
        )
    if self._ping360_config.publish_echo and self.SonarEcho is None:
        self.node.get_logger().warn(
            "ping360_sonar_msgs/msg/SonarEcho is not installed; "
            "/ping360/scan_echo and /ping360/echo are disabled. "
            "Build/source rospkg/src/ping360_sonar_msgs or the upstream ping360_sonar workspace."
        )
    if getattr(self, "_stereo_image_enabled", False):
        render_mode = (
            "sync/reproducible sensor-model"
            if getattr(self, "_camera_sensor_model_enabled", False)
            else (
                "async/interactive"
                if getattr(self, "_stereo_image_async_enabled", False)
                else "sync"
            )
        )
        self.node.get_logger().info(
            "Stereo image bridge active: /stereo/left/image_raw, /stereo/right/image_raw, "
            f"{REAL_CAMERA_RAW_TOPIC}, {REAL_CAMERA_COMPRESSED_TOPIC}, {REAL_CAMERA_INFO_TOPIC}, "
            f"{IMX219_CAMERA0_RAW_TOPIC}, {IMX219_CAMERA0_COMPRESSED_TOPIC}, "
            f"{IMX219_CAMERA0_INFO_TOPIC}, {IMX219_CAMERA1_RAW_TOPIC}, "
            f"{IMX219_CAMERA1_COMPRESSED_TOPIC}, {IMX219_CAMERA1_INFO_TOPIC} "
            f"({self._stereo_image_width}x{self._stereo_image_height}@{self._stereo_image_hz:.1f}Hz "
            f"on demand, render={render_mode})."
        )


__all__ = ["log_ros2_bridge_startup"]
