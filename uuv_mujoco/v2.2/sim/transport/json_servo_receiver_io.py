"""Compatibility facade for ArduPilot JSON-SITL servo transport IO."""

from __future__ import annotations

from .json_servo_receiver_bind import bind_json_servo_socket
from .json_servo_receiver_close import close_json_servo_socket
from .json_servo_receiver_recv import receive_json_servo_packets
from .json_servo_receiver_send import default_json_servo_send_target, send_json_servo_bytes


__all__ = [
    "bind_json_servo_socket",
    "close_json_servo_socket",
    "default_json_servo_send_target",
    "receive_json_servo_packets",
    "send_json_servo_bytes",
]
