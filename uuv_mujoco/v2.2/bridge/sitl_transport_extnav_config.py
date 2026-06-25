"""Compatibility facade for ExternalNav SitlTransport configuration."""

from __future__ import annotations

from bridge.sitl_transport_extnav_base import (
    initialize_extnav_base_state,
    initialize_extnav_origin_state,
    initialize_extnav_vpd_history,
)
from bridge.sitl_transport_extnav_logging import log_extnav_startup_state, log_ignored_extnav_override_envs
from bridge.sitl_transport_extnav_runtime import initialize_extnav_runtime_state
from bridge.sitl_transport_extnav_scheduler import configure_extnav_scheduler


def initialize_extnav_state(transport: object) -> None:
    initialize_extnav_base_state(transport)
    configure_extnav_scheduler(transport)
    initialize_extnav_vpd_history(transport)
    initialize_extnav_origin_state(transport)
    log_ignored_extnav_override_envs()
    initialize_extnav_runtime_state(transport)


__all__ = [
    "configure_extnav_scheduler",
    "initialize_extnav_runtime_state",
    "initialize_extnav_state",
    "log_extnav_startup_state",
]
