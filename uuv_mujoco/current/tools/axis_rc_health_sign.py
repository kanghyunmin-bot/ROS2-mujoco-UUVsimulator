"""Sign-pair health checks for axis RC override reports."""

from __future__ import annotations

import math
from typing import Any

from axis_rc_contract import AXIS_ORDER, EXPECTED_AXIS_METRIC

POSE_RESPONSE_METRIC = {
    "roll": "roll_rad_mean",
    "pitch": "pitch_rad_mean",
    "yaw": "yaw_rad_mean",
}


def sign_pair_checks(summary: list[dict[str, Any]]) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    phase_by_name = {str(row["phase"]): row for row in summary}
    baseline = phase_by_name.get("baseline_neutral")
    for axis in AXIS_ORDER:
        pos = phase_by_name.get(f"{axis}_pos")
        neg = phase_by_name.get(f"{axis}_neg")
        metric = EXPECTED_AXIS_METRIC.get(axis)
        if not pos or not neg or not metric:
            continue
        if _opposite_pose_response(pos, neg, baseline, axis=axis):
            continue
        if _same_response_sign(pos, neg):
            checks.append(
                {
                    "phase": f"{axis}_sign_pair",
                    "axis": axis,
                    "severity": "warn",
                    "flags": ["positive_negative_response_same_sign"],
                }
            )
    return checks


def _opposite_pose_response(
    pos: dict[str, Any],
    neg: dict[str, Any],
    baseline: dict[str, Any] | None,
    *,
    axis: str,
) -> bool:
    metric = POSE_RESPONSE_METRIC.get(axis)
    if not metric:
        return False
    base = _finite_float(baseline.get(metric, 0.0)) if baseline else 0.0
    pos_delta = _finite_float(pos.get(metric, float("nan"))) - base
    neg_delta = _finite_float(neg.get(metric, float("nan"))) - base
    return _opposite_nonzero_sign(pos_delta, neg_delta)


def _same_response_sign(pos: dict[str, Any], neg: dict[str, Any]) -> bool:
    mean_same = _same_nonzero_sign(
        _finite_float(pos.get("expected_metric_mean", float("nan"))),
        _finite_float(neg.get("expected_metric_mean", float("nan"))),
    )
    tail_same = _same_nonzero_sign(
        _finite_float(pos.get("expected_metric_tail_mean", float("nan"))),
        _finite_float(neg.get("expected_metric_tail_mean", float("nan"))),
    )
    mean_available = _finite_nonzero(pos.get("expected_metric_mean")) and _finite_nonzero(
        neg.get("expected_metric_mean")
    )
    tail_available = _finite_nonzero(pos.get("expected_metric_tail_mean")) and _finite_nonzero(
        neg.get("expected_metric_tail_mean")
    )
    if mean_available and tail_available:
        return mean_same and tail_same
    return mean_same or tail_same


def _finite_nonzero(value: Any) -> bool:
    parsed = _finite_float(value)
    return math.isfinite(parsed) and abs(parsed) > 1e-6


def _finite_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _same_nonzero_sign(a: float, b: float) -> bool:
    return math.isfinite(a) and math.isfinite(b) and abs(a) > 1e-6 and abs(b) > 1e-6 and a * b > 0.0


def _opposite_nonzero_sign(a: float, b: float) -> bool:
    return math.isfinite(a) and math.isfinite(b) and abs(a) > 1e-4 and abs(b) > 1e-4 and a * b < 0.0


__all__ = ["sign_pair_checks"]
