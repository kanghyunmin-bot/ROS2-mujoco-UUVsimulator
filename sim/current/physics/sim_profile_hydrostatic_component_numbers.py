"""Numeric validation helpers for hydrostatic profile components."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np


def component_positive_mass(item: Mapping[str, Any]) -> float | None:
    mass = _finite_float(item.get("mass", 0.0))
    if mass is None or mass <= 0.0:
        return None
    return mass


def component_nonnegative_buoyancy_share(item: Mapping[str, Any], *, default: float) -> float | None:
    buoyancy_share = _finite_float(item.get("buoyancy_share", default))
    if buoyancy_share is None or buoyancy_share < 0.0:
        return None
    return buoyancy_share


def _finite_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(parsed):
        return None
    return parsed


__all__ = ["component_nonnegative_buoyancy_share", "component_positive_mass"]
