"""Build hydrostatic body components from simulation-profile entries."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .sim_profile_hydrostatic_component_fields import (
    component_buoyancy_pos,
    component_mass_and_buoyancy,
    component_mass_pos,
    component_name,
    component_size,
)
from .sim_profile_hydrostatic_normalize import normalize_component_shape
from .sim_profile_types import BodyComponent


def body_component_from_profile_item(idx: int, item: Any) -> BodyComponent | None:
    if not isinstance(item, Mapping):
        return None
    size = component_size(item)
    if size is None:
        return None
    mass_pos = component_mass_pos(item)
    if mass_pos is None:
        return None
    buoyancy_pos = component_buoyancy_pos(item, mass_pos)
    mass, buoyancy_share = component_mass_and_buoyancy(item)
    if mass is None or buoyancy_share is None:
        return None
    return BodyComponent(
        name=component_name(idx, item),
        shape=normalize_component_shape(item.get("shape", "ellipsoid")),
        size=size.astype(np.float64, copy=True),
        mass=mass,
        mass_pos=mass_pos.astype(np.float64, copy=True),
        buoyancy_pos=buoyancy_pos.astype(np.float64, copy=True),
        buoyancy_share=buoyancy_share,
    )


__all__ = ["body_component_from_profile_item"]
