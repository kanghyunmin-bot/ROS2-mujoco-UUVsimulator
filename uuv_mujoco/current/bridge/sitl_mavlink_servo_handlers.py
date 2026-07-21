"""Compatibility exports for servo-link MAVLink handlers."""

from __future__ import annotations

from .sitl_mavlink_servo_callback import _call_servo_telemetry_callback
from .sitl_mavlink_servo_heartbeat import (
    _ardupilotmega_value,
    _handle_servo_link_heartbeat,
    _servo_heartbeat_target_ok,
)
from .sitl_mavlink_servo_output import _handle_servo_output_raw, _warn_waiting_for_servo_output


__all__ = [
    "_call_servo_telemetry_callback",
    "_handle_servo_link_heartbeat",
    "_handle_servo_output_raw",
    "_warn_waiting_for_servo_output",
]
