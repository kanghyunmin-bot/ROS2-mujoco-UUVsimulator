"""Compatibility exports for SitlTransport MAVLink runtime helpers."""

from __future__ import annotations

from . import sitl_mavlink_requests, sitl_pwm_runtime
from .sitl_mavlink_connection import (
    _command_mavlink_disabled,
    _connect_sitl_command_mavlink,
    _connect_sitl_mavlink,
    _ensure_command_mavlink_connected,
    _send_gcs_heartbeat,
)
from .sitl_mavlink_polling import _handle_servo_link_heartbeat, _poll_command_mavlink, _poll_servo_mavlink
from .sitl_mavlink_telemetry import _mavlink_source_matches_target, _store_ap_mavlink_telemetry


_handle_pwm_values = sitl_pwm_runtime._handle_pwm_values
_request_sitl_mavlink_servo_stream = sitl_mavlink_requests._request_sitl_mavlink_servo_stream
_request_command_servo_telemetry_stream = sitl_mavlink_requests._request_command_servo_telemetry_stream
_request_sitl_mavlink_ap_telemetry_stream = sitl_mavlink_requests._request_sitl_mavlink_ap_telemetry_stream
_request_command_ap_telemetry_stream = sitl_mavlink_requests._request_command_ap_telemetry_stream


__all__ = [
    "_command_mavlink_disabled",
    "_connect_sitl_command_mavlink",
    "_connect_sitl_mavlink",
    "_ensure_command_mavlink_connected",
    "_handle_pwm_values",
    "_handle_servo_link_heartbeat",
    "_mavlink_source_matches_target",
    "_poll_command_mavlink",
    "_poll_servo_mavlink",
    "_request_command_ap_telemetry_stream",
    "_request_command_servo_telemetry_stream",
    "_request_sitl_mavlink_ap_telemetry_stream",
    "_request_sitl_mavlink_servo_stream",
    "_send_gcs_heartbeat",
    "_store_ap_mavlink_telemetry",
]
