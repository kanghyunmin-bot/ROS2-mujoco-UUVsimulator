"""Compatibility facade for motion/depth/pressure callbacks."""

from __future__ import annotations

from .node_motion_depth_callbacks import (
    _on_atm_pressure,
    _on_bar30_pressure,
    _on_depth,
    _on_pressure_value,
    _on_static_pressure,
)
from .node_motion_imu_callbacks import _on_battery, _on_imu
from .node_motion_pose_callbacks import (
    _on_dvl_odom,
    _on_ground_truth_pose,
    _on_local_odom,
    _on_odom,
    _on_pose,
    _on_rovio_odom,
)
from .node_motion_velocity_callbacks import (
    _on_dvl_velocity,
    _on_velocity,
    _on_velocity_body,
    _on_velocity_local,
)


__all__ = [
    "_on_atm_pressure",
    "_on_bar30_pressure",
    "_on_battery",
    "_on_depth",
    "_on_dvl_odom",
    "_on_dvl_velocity",
    "_on_ground_truth_pose",
    "_on_imu",
    "_on_local_odom",
    "_on_odom",
    "_on_pose",
    "_on_pressure_value",
    "_on_rovio_odom",
    "_on_static_pressure",
    "_on_velocity",
    "_on_velocity_body",
    "_on_velocity_local",
]
