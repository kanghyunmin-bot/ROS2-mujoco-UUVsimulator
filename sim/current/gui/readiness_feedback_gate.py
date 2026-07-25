"""Fresh telemetry feedback gate for GUI command readiness."""

from __future__ import annotations

import math

from .models import TelemetrySnapshot

FRESH_FEEDBACK_MAX_AGE_S = 3.0


def vehicle_state_feedback_reason(snap: TelemetrySnapshot) -> str:
    if not bool(snap.connected):
        return "waiting for fresh vehicle state"
    if not math.isfinite(snap.state_age_s):
        return "waiting for fresh vehicle state"
    if snap.state_age_s >= FRESH_FEEDBACK_MAX_AGE_S:
        return "waiting for fresh vehicle state"
    return ""


def depth_feedback_reason(snap: TelemetrySnapshot) -> str:
    if not math.isfinite(snap.depth_age_s):
        return "waiting for Bar30/depth feedback"
    if snap.depth_age_s >= FRESH_FEEDBACK_MAX_AGE_S:
        return "waiting for fresh Bar30/depth feedback"
    return ""


def imu_feedback_reason(snap: TelemetrySnapshot) -> str:
    if not math.isfinite(snap.imu_age_s):
        return "waiting for IMU feedback"
    if snap.imu_age_s >= FRESH_FEEDBACK_MAX_AGE_S:
        return "waiting for fresh IMU feedback"
    return ""


def feedback_gate_reason(snap: TelemetrySnapshot) -> str:
    return (
        vehicle_state_feedback_reason(snap)
        or depth_feedback_reason(snap)
        or imu_feedback_reason(snap)
    )


__all__ = [
    "FRESH_FEEDBACK_MAX_AGE_S",
    "depth_feedback_reason",
    "feedback_gate_reason",
    "imu_feedback_reason",
    "vehicle_state_feedback_reason",
]
