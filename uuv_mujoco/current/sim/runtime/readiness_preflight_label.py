"""Pre-command readiness label policy."""

from __future__ import annotations

from .readiness_label_types import NOT_READY_STYLE, ReadinessLabel
from .readiness_types import CommandReadinessInputs


def preflight_readiness_label(inputs: CommandReadinessInputs) -> ReadinessLabel | None:
    runtime = inputs.runtime
    if not runtime.vehicle_state_known:
        return "WAIT: vehicle", NOT_READY_STYLE
    if not inputs.real_start_fresh:
        return "WAIT: init state", NOT_READY_STYLE
    if inputs.real_start_required and not inputs.real_start_ok:
        return f"WAIT: init {inputs.real_start_status}", NOT_READY_STYLE
    if float(inputs.settle_left_s) > 0.0:
        return f"WAIT: EKF settle {float(inputs.settle_left_s):.0f}s", NOT_READY_STYLE
    if not inputs.arm_service_ready or not inputs.mode_service_ready:
        return "WAIT: arm/mode", NOT_READY_STYLE
    return None


__all__ = ["preflight_readiness_label"]
