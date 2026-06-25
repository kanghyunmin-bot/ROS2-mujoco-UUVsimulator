"""Axis-response and neutral residual rules for axis RC health checks."""

from __future__ import annotations

import math
from typing import Any

from axis_rc_contract import MIN_EXPECTED_PEAK
from axis_rc_health_flags import apply_failure_flag, apply_warning_flag

MAX_RCIN_ONSET_DELAY_S = 0.10
MAX_RESPONSE_AFTER_RCIN_DELAY_S = 0.75


def check_primary_axis_response(
    row: dict[str, Any],
    flags: list[str],
    severity: str,
    *,
    axis: str,
) -> str:
    if axis == "neutral":
        return severity
    rcin_delay_s = float(row.get("rcin_onset_delay_s", float("nan")))
    if math.isfinite(rcin_delay_s) and rcin_delay_s > MAX_RCIN_ONSET_DELAY_S:
        severity = apply_failure_flag(flags, severity, "slow_rcin_command_echo")
    threshold = MIN_EXPECTED_PEAK.get(axis, 0.0)
    peak = float(row.get("expected_metric_peak_abs", float("nan")))
    if math.isfinite(peak) and peak < threshold:
        return apply_warning_flag(flags, severity, "weak_primary_axis_response")
    response_after_rcin_delay_s = float(row.get("response_after_rcin_delay_s", float("nan")))
    if math.isfinite(response_after_rcin_delay_s) and response_after_rcin_delay_s > MAX_RESPONSE_AFTER_RCIN_DELAY_S:
        return apply_warning_flag(flags, severity, "slow_primary_axis_response")
    return severity


def check_neutral_residuals(
    row: dict[str, Any],
    flags: list[str],
    severity: str,
    *,
    input_mode: str,
) -> str:
    rcin_tail_delta = float(row.get("rcin_tail_max_delta", 0.0))
    if input_mode in ("rc-override", "both") and math.isfinite(rcin_tail_delta) and rcin_tail_delta > 5.0:
        severity = apply_warning_flag(flags, severity, "neutral_rcin_not_released")
    if float(row.get("gyro_z_tail_peak_abs", row.get("gyro_z_peak_abs", 0.0))) > 0.05:
        severity = apply_warning_flag(flags, severity, "high_neutral_yaw_residual")
    if abs(float(row.get("dvl_vz_tail_mean", row.get("dvl_vz_mean", 0.0)))) > 0.12:
        severity = apply_warning_flag(flags, severity, "high_neutral_heave_residual")
    return severity


__all__ = [
    "MAX_RCIN_ONSET_DELAY_S",
    "MAX_RESPONSE_AFTER_RCIN_DELAY_S",
    "check_neutral_residuals",
    "check_primary_axis_response",
]
