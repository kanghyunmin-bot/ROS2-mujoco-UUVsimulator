"""Blocking control-readiness labels."""

from __future__ import annotations

from .readiness_label_types import NOT_READY_STYLE, ReadinessLabel
from .readiness_types import CommandReadinessInputs


def required_mode_label(inputs: CommandReadinessInputs) -> ReadinessLabel | None:
    required_mode = str(inputs.required_mode or "").strip().upper()
    if required_mode and str(inputs.mode or "").strip().upper() != required_mode:
        return f"WAIT: mode {required_mode}", NOT_READY_STYLE
    return None


def control_precondition_label(inputs: CommandReadinessInputs) -> ReadinessLabel | None:
    if not inputs.manual_input:
        return "WAIT: RC link", NOT_READY_STYLE
    if not inputs.armed:
        return "WAIT: arm", NOT_READY_STYLE
    mode_label = required_mode_label(inputs)
    if mode_label is not None:
        return mode_label
    if inputs.real_start_required and not inputs.real_start_released:
        return "WAIT: init release", NOT_READY_STYLE
    return None


__all__ = ["control_precondition_label", "required_mode_label"]
