"""Compatibility exports for MAVLink command target helpers."""

from __future__ import annotations

from .sitl_command_links import (
    _arm_mode_command_pending,
    _command_link_for_mav,
    _mav_for_commands,
    _mav_for_external_nav,
    _mavs_for_arm_mode_commands,
    _resolve_mav_target,
    mavlink_connected,
    rc_override_ready,
)
from .sitl_mavlink_peer import _ensure_mavlink_peer
from .sitl_vehicle_heartbeat import (
    _handle_command_ack,
    _heartbeat_is_vehicle,
    _update_vehicle_heartbeat,
)


__all__ = [
    "mavlink_connected",
    "rc_override_ready",
    "_arm_mode_command_pending",
    "_command_link_for_mav",
    "_ensure_mavlink_peer",
    "_handle_command_ack",
    "_heartbeat_is_vehicle",
    "_mav_for_commands",
    "_mav_for_external_nav",
    "_mavs_for_arm_mode_commands",
    "_resolve_mav_target",
    "_update_vehicle_heartbeat",
]
