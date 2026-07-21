"""Compatibility facade for UuvGuiNode telemetry callback groups."""

from __future__ import annotations

from .node_motion_callbacks import (
    _on_atm_pressure,
    _on_bar30_pressure,
    _on_battery,
    _on_depth,
    _on_dvl_odom,
    _on_filtered_odom,
    _on_dvl_velocity,
    _on_ground_truth_pose,
    _on_imu,
    _on_local_odom,
    _on_odom,
    _on_pose,
    _on_pressure_value,
    _on_rovio_odom,
    _on_static_pressure,
    _on_velocity,
    _on_velocity_body,
    _on_velocity_local,
)
from .node_ping360_callbacks import _on_ping360_status
from .node_rc_callbacks import _on_rc_in, _on_rc_out
from .node_sitl_status_callbacks import _on_real_start_status, _on_sitl_mavlink_telemetry_status
from .node_vehicle_callbacks import _on_state, _on_status_text


__all__ = [
    "_on_atm_pressure",
    "_on_bar30_pressure",
    "_on_battery",
    "_on_depth",
    "_on_dvl_odom",
    "_on_filtered_odom",
    "_on_dvl_velocity",
    "_on_ground_truth_pose",
    "_on_imu",
    "_on_local_odom",
    "_on_odom",
    "_on_ping360_status",
    "_on_pose",
    "_on_pressure_value",
    "_on_rc_in",
    "_on_rc_out",
    "_on_real_start_status",
    "_on_rovio_odom",
    "_on_sitl_mavlink_telemetry_status",
    "_on_state",
    "_on_static_pressure",
    "_on_status_text",
    "_on_velocity",
    "_on_velocity_body",
    "_on_velocity_local",
]
