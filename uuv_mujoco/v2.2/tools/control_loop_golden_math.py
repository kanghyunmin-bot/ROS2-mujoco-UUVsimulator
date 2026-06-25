"""Small numeric helpers for control-loop golden fingerprints."""

from __future__ import annotations

import math
from typing import Any


def finite_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def mean(values: list[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    return sum(finite) / len(finite)


def rms(values: list[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    return math.sqrt(sum(v * v for v in finite) / len(finite))


def max_abs(values: list[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    return max(abs(v) for v in finite)
