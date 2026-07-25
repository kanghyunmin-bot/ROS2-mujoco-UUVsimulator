"""Shared series helpers for ALT_HOLD diagnostics."""

from __future__ import annotations

import math


def finite(values: list[float]) -> list[float]:
    return [float(v) for v in values if math.isfinite(float(v))]


__all__ = ["finite"]
