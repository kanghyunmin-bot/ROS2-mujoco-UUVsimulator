"""Sample, arm, and RC-output movement rules for axis RC health checks."""

from __future__ import annotations

import math
from typing import Any

from axis_rc_contract import MIN_EXPECTED_PEAK
from axis_rc_health_flags import apply_failure_flag, apply_warning_flag


def check_sample_count(
    row: dict[str, Any],
    flags: list[str],
    severity: str,
    *,
    min_samples: int,
) -> str:
    if int(row.get("samples", 0)) < min_samples:
        return apply_failure_flag(flags, severity, "too_few_samples")
    return severity


def check_armed_fraction(row: dict[str, Any], flags: list[str], severity: str) -> str:
    if float(row.get("armed_fraction", 0.0)) < 0.99:
        return apply_failure_flag(flags, severity, "not_fully_armed")
    return severity


def check_rcout_movement(
    row: dict[str, Any],
    flags: list[str],
    severity: str,
    *,
    axis: str,
    input_mode: str,
) -> str:
    if axis != "neutral" and input_mode in ("rc-override", "both"):
        if float(row.get("rcout_max_delta", 0.0)) < 5.0:
            if _rc_input_and_axis_response_present(row, axis=axis):
                return apply_warning_flag(flags, severity, "rcout_telemetry_not_observed_in_phase")
            return apply_failure_flag(flags, severity, "rcout_not_moving")
    return severity


def _rc_input_and_axis_response_present(row: dict[str, Any], *, axis: str) -> bool:
    try:
        rcin_delta = float(row.get("rcin_max_delta", 0.0))
        response_peak = float(row.get("expected_metric_peak_abs", float("nan")))
    except (TypeError, ValueError):
        return False
    response_threshold = float(MIN_EXPECTED_PEAK.get(axis, 0.0))
    return rcin_delta >= 5.0 and math.isfinite(response_peak) and response_peak >= response_threshold


__all__ = ["check_armed_fraction", "check_rcout_movement", "check_sample_count"]
