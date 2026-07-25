"""Component-level buoyancy force calculations for neutral checks."""

from __future__ import annotations

from typing import Any

import numpy as np

from physics.hydrodynamics_helpers import submerged_fraction
from physics_contract_buoyancy import component_half_height, component_share
from physics_contract_neutral_context import NeutralBuoyancyContext


def component_buoyancy(
    base_origin: np.ndarray,
    base_rot: np.ndarray,
    component: Any,
    context: NeutralBuoyancyContext,
) -> tuple[np.ndarray, np.ndarray, float]:
    volume_point_world = base_origin + base_rot @ component.buoyancy_pos.copy()
    force_point_local = component.buoyancy_pos.copy()
    force_point_local[0] += context.cob_longitudinal_offset
    force_point_world = base_origin + base_rot @ force_point_local
    component_depth = float(context.water_surface_z - float(volume_point_world[2]))
    component_submerged = submerged_fraction(
        component_depth * float(context.hydro_cfg.buoyancy_slope_scale),
        component_half_height(component),
        str(context.hydro_cfg.buoyancy_model),
    )
    share = component_share(component, context.total_share)
    component_buoyancy_n = (
        context.rho * context.gravity * context.neutral_volume * share * component_submerged * context.buoyancy_scale
    )
    return np.array([0.0, 0.0, component_buoyancy_n], dtype=np.float64), force_point_world, component_submerged


__all__ = ["component_buoyancy"]
