"""Compatibility exports for ROS2 bridge math and message helpers."""

from __future__ import annotations

from .ros2_math_pressure import pressure_abs_from_depth_m
from .ros2_math_quat import quat_to_yaw, quat_wxyz_to_rotmat, rotmat_to_quat_wxyz
from .ros2_math_rc import clamp_rc_channel, rc_channel_value, rc_to_norm
from .ros2_math_rotation import rpy_deg_to_rotmat
from .ros2_math_scalar import finite_or_zero, wrap_angle_rad
from .ros2_message_setters import set_first_attr, set_nested_xyz

__all__ = [
    "clamp_rc_channel",
    "finite_or_zero",
    "pressure_abs_from_depth_m",
    "quat_to_yaw",
    "quat_wxyz_to_rotmat",
    "rc_channel_value",
    "rc_to_norm",
    "rotmat_to_quat_wxyz",
    "rpy_deg_to_rotmat",
    "set_first_attr",
    "set_nested_xyz",
    "wrap_angle_rad",
]
