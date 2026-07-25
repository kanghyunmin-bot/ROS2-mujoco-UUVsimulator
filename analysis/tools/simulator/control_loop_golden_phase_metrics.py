"""Metric extraction helpers for control-loop golden phase fingerprints."""

from __future__ import annotations

from typing import Any

from control_loop_golden_fingerprint_fields import RESPONSE_METRICS
from control_loop_golden_math import finite_float


def base_phase_metrics(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "axis": str(row.get("axis", "")),
        "command": finite_float(row.get("command")),
    }


def response_phase_metrics(row: dict[str, Any]) -> dict[str, Any]:
    metrics: dict[str, Any] = {}
    for key in RESPONSE_METRICS:
        value = finite_float(row.get(key))
        if value is not None:
            metrics[key] = value
    return metrics


__all__ = ["base_phase_metrics", "response_phase_metrics"]
