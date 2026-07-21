"""Compatibility exports for GUIDED/raw setpoint helpers."""

from __future__ import annotations

from bridge.sitl_body_velocity_setpoint import send_body_velocity_setpoint
from bridge.sitl_local_ned_setpoint import send_position_target_local_ned


__all__ = ["send_body_velocity_setpoint", "send_position_target_local_ned"]
