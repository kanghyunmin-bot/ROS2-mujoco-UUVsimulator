"""MAVLink command, telemetry, and polling configuration for SitlTransport."""

from __future__ import annotations

from bridge.sitl_transport_auto_ready_config import initialize_auto_ready_state
from bridge.sitl_transport_command_link_config import initialize_mavlink_command_path_state
from bridge.sitl_transport_mavlink_base_config import (
    initialize_mavlink_connection_state,
    initialize_mavlink_endpoint_state,
)
from bridge.sitl_transport_mavlink_telemetry_config import initialize_mavlink_telemetry_state
from bridge.sitl_transport_polling_config import initialize_transport_polling_state
from bridge.qgc_mavlink_relay import initialize_qgc_mavlink_relay


def initialize_mavlink_transport(
    transport: object,
    *,
    sitl_mavlink_endpoint: str,
    sitl_mavlink_servo_hz: float,
    sitl_mavlink_target_sysid: int,
    sitl_mavlink_target_compid: int,
    sitl_mavlink_source_sysid: int,
    sitl_mavlink_source_compid: int,
) -> None:
    initialize_mavlink_endpoint_state(
        transport,
        sitl_mavlink_endpoint=sitl_mavlink_endpoint,
        sitl_mavlink_servo_hz=sitl_mavlink_servo_hz,
        sitl_mavlink_target_sysid=sitl_mavlink_target_sysid,
        sitl_mavlink_target_compid=sitl_mavlink_target_compid,
        sitl_mavlink_source_sysid=sitl_mavlink_source_sysid,
        sitl_mavlink_source_compid=sitl_mavlink_source_compid,
    )
    initialize_mavlink_connection_state(transport)
    initialize_mavlink_command_path_state(transport)
    initialize_qgc_mavlink_relay(transport)
    initialize_auto_ready_state(transport)
    transport._sitl_last_disarmed_servo_warn_wall = -1.0
    transport._sitl_last_all_min_servo_warn_wall = -1.0
    initialize_mavlink_telemetry_state(transport)
    initialize_transport_polling_state(transport)


__all__ = [
    "initialize_auto_ready_state",
    "initialize_mavlink_telemetry_state",
    "initialize_mavlink_transport",
    "initialize_transport_polling_state",
    "initialize_qgc_mavlink_relay",
]
