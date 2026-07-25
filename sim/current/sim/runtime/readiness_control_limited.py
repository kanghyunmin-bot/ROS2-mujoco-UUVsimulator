"""Limited-but-commandable readiness labels."""

from __future__ import annotations

from .readiness_label_types import LIMITED_STYLE, ReadinessLabel
from .readiness_types import CommandReadinessInputs


def limited_control_readiness_label(inputs: CommandReadinessInputs) -> ReadinessLabel | None:
    runtime = inputs.runtime
    if not inputs.rc_source_ready:
        return "CMD READY / RC WAIT", LIMITED_STYLE
    if inputs.require_fresh_rcout and not runtime.plant_servo_rows_available:
        return "CMD READY / RCOU WAIT", LIMITED_STYLE
    if not runtime.json_sensor_transport_alive:
        return "CMD READY / DEPTH WAIT", LIMITED_STYLE
    return None


__all__ = ["limited_control_readiness_label"]
