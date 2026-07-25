"""Compatibility facade for SERVO_OUTPUT_RAW request policies."""

from __future__ import annotations

from .sitl_mavlink_request_command_link import request_command_link_servo_stream
from .sitl_mavlink_request_servo_link import request_servo_link_servo_stream


__all__ = ["request_command_link_servo_stream", "request_servo_link_servo_stream"]
