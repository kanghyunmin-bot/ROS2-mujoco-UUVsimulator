"""Polynomial fallback force conversion for thrusters."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from physics.hydrodynamics_helpers import scaled_polynomial_force


def force_from_polynomial_model(
    *,
    name: str,
    command_shaped: float,
    gain: float,
    thruster_global: Mapping[str, Any],
    thruster_force_max: float,
    thruster_reverse_asymmetry: Mapping[str, float | None],
) -> float:
    magnitude = abs(command_shaped)
    if command_shaped >= 0.0:
        force_mag = scaled_polynomial_force(
            magnitude,
            thruster_global["forward_poly"],
            thruster_force_max,
        )
        return float(force_mag * gain)

    reverse_asymmetry = reverse_asymmetry_for_thruster(name, thruster_global, thruster_reverse_asymmetry)
    reverse_force_max = thruster_force_max * float(np.clip(reverse_asymmetry, 0.1, 1.5))
    force_mag = scaled_polynomial_force(
        magnitude,
        thruster_global["reverse_poly"],
        reverse_force_max,
    )
    return float(-force_mag * gain)


def reverse_asymmetry_for_thruster(
    name: str,
    thruster_global: Mapping[str, Any],
    thruster_reverse_asymmetry: Mapping[str, float | None],
) -> float:
    reverse_asymmetry = thruster_reverse_asymmetry.get(name)
    if reverse_asymmetry is None:
        reverse_asymmetry = thruster_global["reverse_asymmetry"]
    return float(reverse_asymmetry)


__all__ = ["force_from_polynomial_model", "reverse_asymmetry_for_thruster"]
