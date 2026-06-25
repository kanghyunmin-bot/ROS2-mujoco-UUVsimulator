"""Parsing helpers for offline thruster performance curves."""

from __future__ import annotations

from typing import Any

from .thruster_performance_values import finite_sorted_curve, parse_voltage, to_float_array


def parse_thruster_performance_curve(curve: Any) -> dict[str, Any] | None:
    if not isinstance(curve, dict):
        return None
    voltage = parse_voltage(curve.get("voltage_v"))
    pwm = to_float_array(curve.get("pwm_us"))
    force = to_float_array(curve.get("force_n"))
    if voltage is None or pwm is None or force is None or pwm.size != force.size or pwm.size < 2:
        return None
    pwm, force = finite_sorted_curve(pwm, force)
    if pwm.size < 2:
        return None
    return {"voltage": voltage, "pwm": pwm, "force": force}


__all__ = [
    "finite_sorted_curve",
    "parse_thruster_performance_curve",
    "parse_voltage",
    "to_float_array",
]
