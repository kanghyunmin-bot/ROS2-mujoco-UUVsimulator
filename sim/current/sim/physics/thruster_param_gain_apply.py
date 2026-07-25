"""Gain-scale application helpers for per-thruster parameters."""

from __future__ import annotations

from typing import MutableMapping

import numpy as np


def clipped_gain_product(local_gain: float, global_gain: float, *, lower: float) -> float:
    value = float(np.clip(float(local_gain), lower, 20.0) * np.clip(float(global_gain), lower, 20.0))
    return float(np.clip(value, lower, 20.0))


def update_thruster_gain(
    values: MutableMapping[str, float],
    name: str,
    new_value: float,
) -> bool:
    changed = abs(float(new_value) - float(values[name])) > 1e-8
    values[name] = float(new_value)
    return changed


__all__ = ["clipped_gain_product", "update_thruster_gain"]
