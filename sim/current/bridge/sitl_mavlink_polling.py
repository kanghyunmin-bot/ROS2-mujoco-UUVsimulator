"""Compatibility exports for SITL MAVLink polling loops."""

from __future__ import annotations

from .sitl_mavlink_command_polling import _handle_command_link_message, _poll_command_mavlink
from .sitl_mavlink_servo_handlers import _handle_servo_link_heartbeat
from .sitl_mavlink_servo_polling import _poll_servo_mavlink


__all__ = ["_handle_servo_link_heartbeat", "_poll_command_mavlink", "_poll_servo_mavlink"]
