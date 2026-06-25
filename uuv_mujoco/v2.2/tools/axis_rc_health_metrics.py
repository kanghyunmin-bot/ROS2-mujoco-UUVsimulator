"""Health gates for axis RC override checks."""

from __future__ import annotations

from typing import Any

from axis_rc_health_phase import phase_health_check
from axis_rc_health_sign import sign_pair_checks
from axis_rc_health_status import overall_health


def build_health(
    summary: list[dict[str, Any]],
    *,
    input_mode: str,
    sample_hz: float,
    axis_s: float,
    neutral_s: float,
    baseline_s: float | None = None,
) -> dict[str, Any]:
    expected_axis_samples = max(1, int(0.55 * float(sample_hz) * float(axis_s)))
    expected_neutral_samples = max(1, int(0.45 * float(sample_hz) * float(neutral_s)))
    expected_baseline_samples = max(
        1,
        int(0.45 * float(sample_hz) * float(baseline_s if baseline_s is not None else neutral_s)),
    )
    checks = [
        phase_health_check(
            row,
            input_mode=input_mode,
            expected_axis_samples=expected_axis_samples,
            expected_neutral_samples=expected_neutral_samples,
            expected_baseline_samples=expected_baseline_samples,
        )
        for row in summary
    ]
    checks.extend(neutral_only_rcout_checks(summary, input_mode=input_mode))
    checks.extend(sign_pair_checks(summary))

    return {"overall": overall_health(checks), "checks": checks}


def neutral_only_rcout_checks(summary: list[dict[str, Any]], *, input_mode: str) -> list[dict[str, Any]]:
    if input_mode not in ("rc-override", "both"):
        return []
    if not summary or any(str(row.get("axis", "")) != "neutral" for row in summary):
        return []
    max_rcin_delta = max((float(row.get("rcin_tail_max_delta", 0.0)) for row in summary), default=0.0)
    if max_rcin_delta >= 5.0:
        return []
    max_rcout_delta = max((float(row.get("rcout_tail_max_delta", 0.0)) for row in summary), default=0.0)
    mean_rcout_delta = max((float(row.get("rcout_tail_mean_abs_delta", 0.0)) for row in summary), default=0.0)
    max_vertical_common = max(
        (abs(float(row.get("rcout_tail_vertical_common_delta", 0.0))) for row in summary),
        default=0.0,
    )
    max_vertical_diff = max(
        (float(row.get("rcout_tail_vertical_diff_max_abs", 0.0)) for row in summary),
        default=0.0,
    )
    max_horizontal_diff = max(
        (float(row.get("rcout_tail_horizontal_diff_max_abs", 0.0)) for row in summary),
        default=0.0,
    )
    flags: list[str] = []
    severity = "ok"
    if max_rcout_delta >= 300.0 or mean_rcout_delta >= 120.0 or max_vertical_common >= 120.0:
        severity = "fail"
        flags.append("neutral_only_rcout_saturated")
    elif max_vertical_common >= 50.0:
        severity = "warn"
        flags.append("neutral_only_heave_common_active")
    elif max_vertical_diff >= 50.0 or max_horizontal_diff >= 50.0 or max_rcout_delta >= 50.0 or mean_rcout_delta >= 25.0:
        severity = "warn"
        flags.append("neutral_only_rcout_attitude_differential")
    if severity == "ok":
        return []
    return [
        {
            "phase": "neutral_only_rcout_contract",
            "axis": "neutral",
            "severity": severity,
            "flags": flags,
            "max_rcout_tail_delta": max_rcout_delta,
            "max_rcout_tail_mean_abs_delta": mean_rcout_delta,
            "max_rcout_tail_vertical_common_delta": max_vertical_common,
            "max_rcout_tail_vertical_diff_max_abs": max_vertical_diff,
            "max_rcout_tail_horizontal_diff_max_abs": max_horizontal_diff,
        }
    ]


__all__ = ["build_health", "neutral_only_rcout_checks"]
