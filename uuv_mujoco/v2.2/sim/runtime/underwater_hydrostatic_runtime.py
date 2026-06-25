"""Hydrostatic buoyancy and restoring-torque runtime helpers."""

from __future__ import annotations

from typing import Any

import numpy as np

from physics.hydrodynamics_helpers import submerged_fraction
from sim.runtime.underwater_hydrostatic_restoring import restoring_tau_world
from sim.runtime.underwater_hydrostatic_weighted import (
    body_components_wrench,
    buoyancy_points_wrench,
    weighted_hydrostatic_result,
)
from sim.runtime.underwater_wrench_types import HydrostaticWrenchResult


def apply_hydrostatic_wrench(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    base_origin: np.ndarray,
    com: np.ndarray,
) -> HydrostaticWrenchResult:
    hs = runtime.hydrostatic
    hyd = runtime.hydrodynamics
    data = runtime.data

    cob = data.site_xpos[hs.cob_site_id].copy() if hs.cob_site_id >= 0 else com
    depth = runtime.water_surface_z - float(base_origin[2])
    submerged = submerged_fraction(depth, hyd.half_height, hyd.buoyancy_model)
    buoyancy_submerged = submerged_fraction(
        depth * hyd.buoyancy_slope_scale,
        hyd.half_height,
        hyd.buoyancy_model,
    )
    buoy_tau_world = np.zeros(3, dtype=np.float64)
    buoy_force_world = np.zeros(3, dtype=np.float64)
    buoy_point = cob.copy()

    if hs.active_buoyancy_points:
        result = buoyancy_points_wrench(runtime, base_rot=base_rot, base_origin=base_origin, com=com, cob=cob)
        submerged = result.submerged
        buoyancy_submerged = result.buoyancy_submerged
        buoy_force_world = result.buoy_force_world
        buoy_tau_world = result.buoy_tau_world
        buoy_point = result.buoy_point_world
    elif hs.active_body_components:
        result = body_components_wrench(runtime, base_rot=base_rot, base_origin=base_origin, com=com, cob=cob)
        submerged = result.submerged
        buoyancy_submerged = result.buoyancy_submerged
        buoy_force_world = result.buoy_force_world
        buoy_tau_world = result.buoy_tau_world
        buoy_point = result.buoy_point_world
    else:
        buoyancy_blend = hyd.buoyancy_point_blend * buoyancy_submerged
        buoy_point = ((1.0 - buoyancy_blend) * com) + (buoyancy_blend * cob)
        buoy = runtime.rho * runtime.gravity * hyd.neutral_volume * buoyancy_submerged * hyd.buoyancy_scale
        buoy_force_world = np.array([0.0, 0.0, buoy], dtype=np.float64)
        if abs(hyd.cob_torque_scale) > 1e-9:
            buoy_tau_world = np.cross(buoy_point - com, buoy_force_world) * hyd.cob_torque_scale

    if hs.hydrostatic_restoring_active:
        buoy_tau_world += restoring_tau_world(runtime, base_rot)

    return HydrostaticWrenchResult(
        submerged=float(submerged),
        buoyancy_submerged=float(buoyancy_submerged),
        buoy_force_world=buoy_force_world,
        buoy_tau_world=buoy_tau_world,
        buoy_point_world=buoy_point,
    )


_body_components_wrench = body_components_wrench
_buoyancy_points_wrench = buoyancy_points_wrench
_restoring_tau_world = restoring_tau_world
_weighted_hydrostatic_result = weighted_hydrostatic_result


__all__ = [
    "_body_components_wrench",
    "_buoyancy_points_wrench",
    "_restoring_tau_world",
    "_weighted_hydrostatic_result",
    "apply_hydrostatic_wrench",
]
