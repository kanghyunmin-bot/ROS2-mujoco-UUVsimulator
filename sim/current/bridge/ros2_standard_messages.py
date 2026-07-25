"""Compatibility facade for common ROS2 message builders."""

from __future__ import annotations

from .ros2_pose_messages import build_depth_pose_msg, build_odom_msg, build_pose_msg
from .ros2_sensor_messages import (
    apply_real_mavros_imu_covariance,
    build_battery_msg,
    build_imu_msg,
    build_pressure_msg,
    build_range_msg,
    build_vfr_hud_msg,
)
from .ros2_status_messages import build_json_string_msg
from .ros2_twist_messages import build_twist_cov_msg, build_twist_msg


__all__ = [
    "build_pose_msg",
    "build_twist_msg",
    "build_twist_cov_msg",
    "build_imu_msg",
    "apply_real_mavros_imu_covariance",
    "build_depth_pose_msg",
    "build_odom_msg",
    "build_pressure_msg",
    "build_range_msg",
    "build_json_string_msg",
    "build_battery_msg",
    "build_vfr_hud_msg",
]
