"""Compatibility exports for MAVLink connection and heartbeat helpers."""

from __future__ import annotations

from .sitl_mavlink_command_connection import (
    _command_mavlink_disabled,
    _connect_sitl_command_mavlink,
    _ensure_command_mavlink_connected,
)
from .sitl_mavlink_heartbeat import _send_gcs_heartbeat
from .sitl_mavlink_servo_connection import _connect_sitl_mavlink


__all__ = [
    "_command_mavlink_disabled",
    "_connect_sitl_command_mavlink",
    "_connect_sitl_mavlink",
    "_ensure_command_mavlink_connected",
    "_send_gcs_heartbeat",
]
