"""Value conversion helpers for real-start status payloads."""

from __future__ import annotations

import math


def finite_float_or_nan(value: float) -> float:
    return float(value) if math.isfinite(value) else math.nan


__all__ = ["finite_float_or_nan"]
