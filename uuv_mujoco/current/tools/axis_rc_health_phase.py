"""Per-phase health checks for axis RC override reports."""

from __future__ import annotations

from typing import Any

from axis_rc_health_rules import (
    check_armed_fraction,
    check_neutral_residuals,
    check_primary_axis_response,
    check_rcout_movement,
    check_sample_count,
)


def phase_health_check(
    row: dict[str, Any],
    *,
    input_mode: str,
    expected_axis_samples: int,
    expected_neutral_samples: int,
    expected_baseline_samples: int,
) -> dict[str, Any]:
    phase = str(row["phase"])
    axis = str(row["axis"])
    flags: list[str] = []
    severity = "ok"
    if axis != "neutral":
        min_samples = expected_axis_samples
    elif phase == "baseline_neutral":
        min_samples = expected_baseline_samples
    else:
        min_samples = expected_neutral_samples
    severity = check_sample_count(row, flags, severity, min_samples=min_samples)
    severity = check_armed_fraction(row, flags, severity)
    severity = check_rcout_movement(row, flags, severity, axis=axis, input_mode=input_mode)
    if axis != "neutral":
        severity = check_primary_axis_response(row, flags, severity, axis=axis)
    else:
        severity = check_neutral_residuals(row, flags, severity, input_mode=input_mode)
    return {"phase": phase, "axis": axis, "severity": severity, "flags": flags}


__all__ = ["phase_health_check"]
