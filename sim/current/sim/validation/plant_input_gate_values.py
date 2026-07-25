"""Value parsing helpers for plant-input validation gates."""

from __future__ import annotations

import math


def finite_float(value: object) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def non_neutral_pwm_values(
    values: list[float],
    *,
    neutral_center: float,
    neutral_tolerance: float,
) -> bool:
    finite_values = [value for value in values if math.isfinite(value)]
    return bool(
        finite_values
        and any(abs(value - neutral_center) > neutral_tolerance for value in finite_values)
    )


__all__ = ["finite_float", "non_neutral_pwm_values"]
