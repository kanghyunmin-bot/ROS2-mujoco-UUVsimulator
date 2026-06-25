"""Numeric helpers for offline thruster performance curve parsing."""

from __future__ import annotations

from typing import Any

import numpy as np


def to_float_array(values: Any) -> np.ndarray | None:
    if not isinstance(values, list) or not values:
        return None
    try:
        out = np.array([float(value) for value in values], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(out)):
        return None
    return out


def parse_voltage(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def finite_sorted_curve(pwm: np.ndarray, force: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(pwm)
    pwm = pwm[order]
    force = force[order]
    valid = np.isfinite(pwm) & np.isfinite(force)
    if np.sum(valid) < 2:
        return np.array([], dtype=np.float64), np.array([], dtype=np.float64)
    return pwm[valid], force[valid]


__all__ = ["finite_sorted_curve", "parse_voltage", "to_float_array"]
