"""Compatibility exports for SitlTransport MAVLink command helpers."""

from __future__ import annotations

from . import sitl_command_targets
from .sitl_arm_mode_runtime import (
    _mode_id_for_text,
    _send_arm_disarm_mavlink,
    _send_set_mode_mavlink,
    _service_pending_arm_command,
    _service_pending_mode_command,
    queue_arm_command,
    queue_set_mode,
    send_arm_command,
    send_set_mode,
)
from .sitl_auto_ready_runtime import (
    _auto_ready_extnav_ready,
    _neutral_rc_values,
    _send_auto_ready_neutral_rc,
    _service_auto_ready_sequence,
    _set_auto_ready_state,
)
from .sitl_rc_manual_runtime import (
    _normalize_rc_override_values,
    _send_neutral_rc_keepalive,
    _send_rc_channels_override,
    _warn_rc_override_not_forwarded,
    send_body_velocity_setpoint,
    send_manual_control,
    send_position_target_local_ned,
    send_rc_override,
)


mavlink_connected = sitl_command_targets.mavlink_connected
rc_override_ready = sitl_command_targets.rc_override_ready
_mav_for_commands = sitl_command_targets._mav_for_commands
_mav_for_external_nav = sitl_command_targets._mav_for_external_nav
_mavs_for_arm_mode_commands = sitl_command_targets._mavs_for_arm_mode_commands
_arm_mode_command_pending = sitl_command_targets._arm_mode_command_pending
_command_link_for_mav = sitl_command_targets._command_link_for_mav
_resolve_mav_target = sitl_command_targets._resolve_mav_target
_heartbeat_is_vehicle = sitl_command_targets._heartbeat_is_vehicle
_update_vehicle_heartbeat = sitl_command_targets._update_vehicle_heartbeat
_handle_command_ack = sitl_command_targets._handle_command_ack
_ensure_mavlink_peer = sitl_command_targets._ensure_mavlink_peer


__all__ = [
    "mavlink_connected",
    "rc_override_ready",
    "_neutral_rc_values",
    "_auto_ready_extnav_ready",
    "_set_auto_ready_state",
    "_send_auto_ready_neutral_rc",
    "_service_auto_ready_sequence",
    "_normalize_rc_override_values",
    "_send_rc_channels_override",
    "_mav_for_commands",
    "_mav_for_external_nav",
    "_mavs_for_arm_mode_commands",
    "_arm_mode_command_pending",
    "_command_link_for_mav",
    "_resolve_mav_target",
    "_send_arm_disarm_mavlink",
    "_mode_id_for_text",
    "_send_set_mode_mavlink",
    "queue_arm_command",
    "queue_set_mode",
    "_service_pending_arm_command",
    "_service_pending_mode_command",
    "_heartbeat_is_vehicle",
    "_update_vehicle_heartbeat",
    "_handle_command_ack",
    "_ensure_mavlink_peer",
    "send_rc_override",
    "_send_neutral_rc_keepalive",
    "send_manual_control",
    "_warn_rc_override_not_forwarded",
    "send_arm_command",
    "send_set_mode",
    "send_body_velocity_setpoint",
    "send_position_target_local_ned",
]
