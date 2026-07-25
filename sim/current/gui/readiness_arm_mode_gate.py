"""Arm/mode command blocking rules for the GUI."""

from __future__ import annotations

from typing import Optional

from .models import TelemetrySnapshot
from .readiness_command_link import sitl_mavlink_command_alive
from .readiness_feedback_gate import (
    FRESH_FEEDBACK_MAX_AGE_S,
    depth_feedback_reason,
    feedback_gate_reason,
    imu_feedback_reason,
    vehicle_state_feedback_reason,
)


def request_is_always_allowed(*, arm_value: Optional[bool], mode: str) -> bool:
    if arm_value is False:
        return True
    requested_mode = str(mode or "").upper()
    return requested_mode in {"", "MANUAL"} and arm_value is None


def arm_mode_gate_reason(
    backend: str,
    snap: TelemetrySnapshot,
    *,
    settle_left_s: float,
    arm_value: Optional[bool] = None,
    mode: str = "",
) -> str:
    """Return a blocking reason for GUI arm/mode commands, or empty string."""
    if request_is_always_allowed(arm_value=arm_value, mode=mode):
        return ""

    command_alive = sitl_mavlink_command_alive(backend, snap)
    vehicle_reason = vehicle_state_feedback_reason(snap)
    if vehicle_reason and not command_alive:
        return vehicle_reason
    feedback_reason = depth_feedback_reason(snap) or imu_feedback_reason(snap)
    if feedback_reason:
        return feedback_reason
    if not command_alive:
        return "waiting for SITL MAVLink command link"
    if settle_left_s > 0.0:
        return f"waiting EKF/ExternalNav settle ({settle_left_s:.1f}s left)"
    return ""


__all__ = [
    "FRESH_FEEDBACK_MAX_AGE_S",
    "arm_mode_gate_reason",
    "feedback_gate_reason",
    "request_is_always_allowed",
]
