"""Operator-control readiness label policy."""

from __future__ import annotations

from .readiness_control_limited import limited_control_readiness_label
from .readiness_control_preconditions import control_precondition_label
from .readiness_label_types import READY_STYLE, ReadinessLabel
from .readiness_types import CommandReadinessInputs


def control_readiness_label(inputs: CommandReadinessInputs) -> ReadinessLabel:
    for stage in (control_precondition_label, limited_control_readiness_label):
        label = stage(inputs)
        if label is not None:
            return label
    return "READY", READY_STYLE


__all__ = ["control_readiness_label"]
