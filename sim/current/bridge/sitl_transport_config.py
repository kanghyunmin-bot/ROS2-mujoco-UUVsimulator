"""Compatibility facade for SitlTransport construction configuration."""

from __future__ import annotations

from bridge.sitl_transport_control_config import initialize_sitl_control_state
from bridge.sitl_transport_extnav_config import (
    configure_extnav_scheduler,
    initialize_extnav_runtime_state,
    initialize_extnav_state,
    log_extnav_startup_state,
)
from bridge.sitl_transport_json_config import initialize_json_servo_transport
from bridge.sitl_transport_mavlink_config import (
    initialize_auto_ready_state,
    initialize_mavlink_telemetry_state,
    initialize_mavlink_transport,
    initialize_transport_polling_state,
)

__all__ = [
    "configure_extnav_scheduler",
    "initialize_auto_ready_state",
    "initialize_extnav_runtime_state",
    "initialize_extnav_state",
    "initialize_json_servo_transport",
    "initialize_mavlink_telemetry_state",
    "initialize_mavlink_transport",
    "initialize_sitl_control_state",
    "initialize_transport_polling_state",
    "log_extnav_startup_state",
]
