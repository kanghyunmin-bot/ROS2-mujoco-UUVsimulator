"""Field groups for global thruster parameter parsing."""

from __future__ import annotations

from typing import Any, MutableMapping

import numpy as np

from .thruster_param_common import is_number


GLOBAL_SCALAR_KEYS = (
    "deadzone",
    "tau_up",
    "tau_down",
    "reverse_asymmetry",
    "command_limit",
    "reaction_torque_gain",
    "inflow_reference_speed_mps",
    "inflow_minimum_reference_speed_mps",
    "inflow_command_exponent",
    "inflow_gain_per_advance_ratio",
    "inflow_minimum_multiplier",
    "inflow_maximum_multiplier",
)
GLOBAL_BOOL_KEYS = ("inflow_enabled",)
GLOBAL_POLY_KEYS = ("forward_poly", "reverse_poly")


def apply_global_scalar_fields(global_cfg: dict[str, Any], thruster_global: MutableMapping[str, Any]) -> None:
    for key in GLOBAL_SCALAR_KEYS:
        value = global_cfg.get(key)
        if is_number(value):
            thruster_global[key] = float(value)


def apply_global_bool_fields(global_cfg: dict[str, Any], thruster_global: MutableMapping[str, Any]) -> None:
    for key in GLOBAL_BOOL_KEYS:
        value = global_cfg.get(key)
        if isinstance(value, bool):
            thruster_global[key] = value


def clipped_global_scale(value: object, *, default: float, lower: float, upper: float) -> float:
    if not is_number(value):
        return default
    return float(np.clip(float(value), lower, upper))


def apply_global_poly_fields(global_cfg: dict[str, Any], thruster_global: MutableMapping[str, Any]) -> None:
    for key in GLOBAL_POLY_KEYS:
        coeffs = global_cfg.get(key)
        if isinstance(coeffs, list) and coeffs:
            try:
                thruster_global[key] = [float(item) for item in coeffs]
            except (TypeError, ValueError):
                pass


__all__ = [
    "GLOBAL_POLY_KEYS",
    "GLOBAL_BOOL_KEYS",
    "GLOBAL_SCALAR_KEYS",
    "apply_global_poly_fields",
    "apply_global_bool_fields",
    "apply_global_scalar_fields",
    "clipped_global_scale",
]
