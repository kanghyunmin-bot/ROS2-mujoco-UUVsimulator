"""MAVLink connection, request, and polling bindings for SitlTransport."""

from __future__ import annotations

from bridge import sitl_mavlink_runtime


class SitlTransportMavlinkBindings:
    _connect_sitl_mavlink = sitl_mavlink_runtime._connect_sitl_mavlink
    _command_mavlink_disabled = sitl_mavlink_runtime._command_mavlink_disabled
    _connect_sitl_command_mavlink = sitl_mavlink_runtime._connect_sitl_command_mavlink
    _ensure_command_mavlink_connected = sitl_mavlink_runtime._ensure_command_mavlink_connected
    _send_gcs_heartbeat = sitl_mavlink_runtime._send_gcs_heartbeat
    _request_sitl_mavlink_servo_stream = sitl_mavlink_runtime._request_sitl_mavlink_servo_stream
    _request_command_servo_telemetry_stream = sitl_mavlink_runtime._request_command_servo_telemetry_stream
    _request_sitl_mavlink_ap_telemetry_stream = sitl_mavlink_runtime._request_sitl_mavlink_ap_telemetry_stream
    _request_command_ap_telemetry_stream = sitl_mavlink_runtime._request_command_ap_telemetry_stream
    _mavlink_source_matches_target = sitl_mavlink_runtime._mavlink_source_matches_target
    _store_ap_mavlink_telemetry = sitl_mavlink_runtime._store_ap_mavlink_telemetry
    _handle_pwm_values = sitl_mavlink_runtime._handle_pwm_values
    _handle_servo_link_heartbeat = sitl_mavlink_runtime._handle_servo_link_heartbeat
    _poll_servo_mavlink = sitl_mavlink_runtime._poll_servo_mavlink
    _poll_command_mavlink = sitl_mavlink_runtime._poll_command_mavlink


__all__ = ["SitlTransportMavlinkBindings"]
