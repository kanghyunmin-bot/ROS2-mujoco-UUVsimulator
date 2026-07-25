"""Runtime parsing helpers for thruster performance curves."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.parsing import to_float_array


def parse_performance_curve(curve: Any) -> dict[str, Any] | None:
    if not isinstance(curve, dict):
        return None
    voltage = curve.get("voltage_v")
    pwm = to_float_array(curve.get("pwm_us"))
    force = to_float_array(curve.get("force_n"))
    if not raw_curve_is_usable(voltage, pwm, force):
        return None
    pwm, force = finite_sorted_curve(pwm, force)
    if pwm.size == 0:
        return None
    return {
        "voltage": float(voltage),
        "pwm": pwm,
        "force": force,
    }


def raw_curve_is_usable(voltage: Any, pwm: np.ndarray | None, force: np.ndarray | None) -> bool:
    if voltage is None or pwm is None or force is None:
        return False
    if pwm.size != force.size:
        return False
    return bool(pwm.size >= 2)


def finite_sorted_curve(pwm: np.ndarray, force: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(pwm)
    pwm = pwm[order]
    force = force[order]
    valid = np.isfinite(pwm) & np.isfinite(force)
    if not np.any(valid):
        return np.array([], dtype=np.float64), np.array([], dtype=np.float64)
    return pwm[valid], force[valid]


__all__ = ["finite_sorted_curve", "parse_performance_curve", "raw_curve_is_usable"]
