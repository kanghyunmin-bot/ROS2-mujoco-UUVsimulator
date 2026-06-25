"""Compatibility facade for UuvGuiNode command helpers."""

from __future__ import annotations

from .node_arm_mode_commands import (
    _on_arm_response,
    _on_mode_response,
    _send_arm_request,
    _send_mode_request,
    arm,
    set_mode,
)
from .node_commanding_common import (
    _arm_mode_gate_reason,
    _arm_mode_settle_left_s,
    _arm_target_reached,
    _call_trigger_service,
    _fresh_vehicle_state,
    _mode_target_reached,
    _publish_command_override,
    _request_initial_depth_release_when_armed,
    _retry_arm_request,
    _retry_mode_request,
    _schedule_once,
    _state_age_s,
    _try_release_initial_depth_hold,
    _vehicle_ready_for_initial_depth_release,
    request_initial_depth_release_when_armed,
)
from .node_rc_publishers import (
    publish_manual_control,
    publish_ping360_config,
    publish_ping360_enabled,
    publish_rc_channels,
    publish_rc_override,
    publish_rc_release,
)

__all__ = [
    "_call_trigger_service",
    "_vehicle_ready_for_initial_depth_release",
    "_try_release_initial_depth_hold",
    "_request_initial_depth_release_when_armed",
    "request_initial_depth_release_when_armed",
    "_schedule_once",
    "_state_age_s",
    "_fresh_vehicle_state",
    "_arm_mode_settle_left_s",
    "_arm_mode_gate_reason",
    "_arm_target_reached",
    "_mode_target_reached",
    "_retry_arm_request",
    "_retry_mode_request",
    "_publish_command_override",
    "_send_arm_request",
    "arm",
    "_on_arm_response",
    "set_mode",
    "_send_mode_request",
    "_on_mode_response",
    "publish_rc_override",
    "publish_manual_control",
    "publish_rc_release",
    "publish_rc_channels",
    "publish_ping360_config",
    "publish_ping360_enabled",
]
