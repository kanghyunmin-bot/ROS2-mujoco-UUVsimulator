"""Compatibility facade for SITL RC, MANUAL_CONTROL, and setpoint helpers."""

from __future__ import annotations

from bridge.sitl_guided_setpoint_runtime import (
    send_body_velocity_setpoint,
    send_position_target_local_ned,
)
from bridge.sitl_manual_control_runtime import send_manual_control
from bridge.sitl_rc_override_runtime import (
    _normalize_rc_override_values,
    _send_neutral_rc_keepalive,
    _send_rc_channels_override,
    _warn_rc_override_not_forwarded,
    send_rc_override,
)

__all__ = [
    "_normalize_rc_override_values",
    "_send_rc_channels_override",
    "send_rc_override",
    "_send_neutral_rc_keepalive",
    "send_manual_control",
    "_warn_rc_override_not_forwarded",
    "send_body_velocity_setpoint",
    "send_position_target_local_ned",
]
