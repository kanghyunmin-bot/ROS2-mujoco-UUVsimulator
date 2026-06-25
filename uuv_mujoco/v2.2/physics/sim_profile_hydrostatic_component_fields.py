"""Field extraction helpers for profile body components."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .sim_profile_hydrostatic_component_numbers import (
    component_nonnegative_buoyancy_share,
    component_positive_mass,
)
from .sim_profile_parse_common import to_float_array


def component_size(item: Mapping[str, Any]) -> np.ndarray | None:
    size = to_float_array(item.get("size"), 3)
    if size is None or np.any(size <= 0.0):
        return None
    return size


def component_mass_pos(item: Mapping[str, Any]) -> np.ndarray | None:
    mass_pos = to_float_array(item.get("mass_pos"), 3)
    if mass_pos is None:
        mass_pos = to_float_array(item.get("pos"), 3)
    return mass_pos


def component_buoyancy_pos(item: Mapping[str, Any], mass_pos: np.ndarray) -> np.ndarray:
    buoyancy_pos = to_float_array(item.get("buoyancy_pos"), 3)
    if buoyancy_pos is None:
        buoyancy_pos = mass_pos.copy()
    return buoyancy_pos


def component_mass_and_buoyancy(item: Mapping[str, Any]) -> tuple[float | None, float | None]:
    mass = component_positive_mass(item)
    if mass is None:
        return None, None
    buoyancy_share = component_nonnegative_buoyancy_share(item, default=mass)
    if buoyancy_share is None:
        return None, None
    return mass, buoyancy_share


def component_name(idx: int, item: Mapping[str, Any]) -> str:
    return str(item.get("name", f"component_{idx}")).strip() or f"component_{idx}"


__all__ = [
    "component_buoyancy_pos",
    "component_mass_and_buoyancy",
    "component_mass_pos",
    "component_name",
    "component_size",
]
