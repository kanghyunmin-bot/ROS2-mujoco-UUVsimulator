"""Thruster command shaping and polynomial force helpers."""

from __future__ import annotations

from typing import Iterable

import numpy as np


def shape_thruster_command(command: float, deadzone: float, command_limit: float) -> float:
    """Apply deadzone and saturation, returning a normalized command in [-1, 1]."""

    deadzone = float(np.clip(deadzone, 0.0, 0.95))
    command_limit = float(np.clip(command_limit, deadzone + 1e-3, 1.0))
    signed = float(np.clip(command, -1.0, 1.0))
    sign = -1.0 if signed < 0.0 else 1.0
    magnitude = min(abs(signed), command_limit)
    if magnitude <= deadzone:
        return 0.0
    # A command limit caps input; it must not stretch a partial command back
    # to full thrust when the limit is reduced.
    span = max(1.0 - deadzone, 1e-6)
    return sign * float(np.clip((magnitude - deadzone) / span, 0.0, 1.0))


def polyval_ascending(coeffs: Iterable[float], x: float) -> float:
    total = 0.0
    power = 1.0
    for coeff in coeffs:
        total += float(coeff) * power
        power *= x
    return total


def scaled_polynomial_force(magnitude: float, coeffs: Iterable[float], force_max: float) -> float:
    magnitude = float(np.clip(magnitude, 0.0, 1.0))
    coeff_list = list(coeffs)
    if not coeff_list:
        return force_max * magnitude
    raw = polyval_ascending(coeff_list, magnitude)
    raw_full = polyval_ascending(coeff_list, 1.0)
    if abs(raw_full) < 1e-9:
        return force_max * magnitude
    return float(force_max * raw / raw_full)


__all__ = ["polyval_ascending", "scaled_polynomial_force", "shape_thruster_command"]
