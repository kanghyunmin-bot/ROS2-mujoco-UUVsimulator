"""MAVLink command and RC forwarding bindings for SitlTransport."""

from __future__ import annotations

from bridge import sitl_commanding


class SitlTransportCommandBindings:
    mavlink_connected = sitl_commanding.mavlink_connected
    rc_override_ready = sitl_commanding.rc_override_ready
    _neutral_rc_values = sitl_commanding._neutral_rc_values
    _auto_ready_extnav_ready = sitl_commanding._auto_ready_extnav_ready
    _set_auto_ready_state = sitl_commanding._set_auto_ready_state
    _send_auto_ready_neutral_rc = sitl_commanding._send_auto_ready_neutral_rc
    _service_auto_ready_sequence = sitl_commanding._service_auto_ready_sequence
    _normalize_rc_override_values = sitl_commanding._normalize_rc_override_values
    _send_rc_channels_override = sitl_commanding._send_rc_channels_override
    _mav_for_commands = sitl_commanding._mav_for_commands
    _mav_for_external_nav = sitl_commanding._mav_for_external_nav
    _mavs_for_arm_mode_commands = sitl_commanding._mavs_for_arm_mode_commands
    _arm_mode_command_pending = sitl_commanding._arm_mode_command_pending
    _command_link_for_mav = sitl_commanding._command_link_for_mav
    _resolve_mav_target = sitl_commanding._resolve_mav_target
    _send_arm_disarm_mavlink = sitl_commanding._send_arm_disarm_mavlink
    _mode_id_for_text = sitl_commanding._mode_id_for_text
    _send_set_mode_mavlink = sitl_commanding._send_set_mode_mavlink
    queue_arm_command = sitl_commanding.queue_arm_command
    queue_set_mode = sitl_commanding.queue_set_mode
    _service_pending_arm_command = sitl_commanding._service_pending_arm_command
    _service_pending_mode_command = sitl_commanding._service_pending_mode_command
    _heartbeat_is_vehicle = sitl_commanding._heartbeat_is_vehicle
    _update_vehicle_heartbeat = sitl_commanding._update_vehicle_heartbeat
    _handle_command_ack = sitl_commanding._handle_command_ack
    _ensure_mavlink_peer = sitl_commanding._ensure_mavlink_peer
    send_rc_override = sitl_commanding.send_rc_override
    _send_neutral_rc_keepalive = sitl_commanding._send_neutral_rc_keepalive
    send_manual_control = sitl_commanding.send_manual_control
    _warn_rc_override_not_forwarded = sitl_commanding._warn_rc_override_not_forwarded
    send_arm_command = sitl_commanding.send_arm_command
    send_set_mode = sitl_commanding.send_set_mode
    send_body_velocity_setpoint = sitl_commanding.send_body_velocity_setpoint
    send_position_target_local_ned = sitl_commanding.send_position_target_local_ned


__all__ = ["SitlTransportCommandBindings"]
