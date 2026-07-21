"""Scalar predicates for real-start status checks."""

from __future__ import annotations

import math
from collections.abc import Iterable


def all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def missing_or_abs_exceeds(value: float, tolerance: float) -> bool:
    return not math.isfinite(value) or abs(value) > tolerance


def missing_or_exceeds(value: float, tolerance: float) -> bool:
    return not math.isfinite(value) or value > tolerance


def finite_and_exceeds(value: float, tolerance: float) -> bool:
    return math.isfinite(value) and value > tolerance


def finite_abs_exceeds(value: float, tolerance: float) -> bool:
    return math.isfinite(value) and abs(value) > tolerance


__all__ = [
    "all_finite",
    "finite_abs_exceeds",
    "finite_and_exceeds",
    "missing_or_abs_exceeds",
    "missing_or_exceeds",
]
