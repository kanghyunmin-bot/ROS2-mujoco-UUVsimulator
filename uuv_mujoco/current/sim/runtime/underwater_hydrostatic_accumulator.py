"""Weighted hydrostatic force accumulation for runtime buoyancy samples."""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np

from physics.hydrodynamics_helpers import submerged_fraction
from sim.runtime.underwater_hydrostatic_samples import WeightedHydrostaticSample
from sim.runtime.underwater_wrench_types import HydrostaticWrenchResult


def accumulate_weighted_hydrostatic(
    runtime: Any,
    samples: Iterable[WeightedHydrostaticSample],
    *,
    base_rot: np.ndarray,
    base_origin: np.ndarray,
    com: np.ndarray,
    cob: np.ndarray,
) -> HydrostaticWrenchResult:
    hyd = runtime.hydrodynamics
    weighted_submerged = 0.0
    weighted_buoyancy_submerged = 0.0
    weighted_point = np.zeros(3, dtype=np.float64)
    buoy_force_world = np.zeros(3, dtype=np.float64)
    buoy_tau_world = np.zeros(3, dtype=np.float64)

    for sample in samples:
        force_world, force_point_world, submerged, buoyancy_submerged = _sample_force(
            runtime,
            sample,
            base_rot=base_rot,
            base_origin=base_origin,
        )
        buoy_force_world += force_world
        weighted_submerged += sample.share * submerged
        weighted_buoyancy_submerged += sample.share * buoyancy_submerged
        weighted_point += force_world[2] * force_point_world
        if abs(hyd.cob_torque_scale) > 1e-9:
            buoy_tau_world += np.cross(force_point_world - com, force_world) * hyd.cob_torque_scale

    return weighted_hydrostatic_result(
        weighted_submerged,
        weighted_buoyancy_submerged,
        weighted_point,
        buoy_force_world,
        buoy_tau_world,
        cob,
    )


def _sample_force(
    runtime: Any,
    sample: WeightedHydrostaticSample,
    *,
    base_rot: np.ndarray,
    base_origin: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    hyd = runtime.hydrodynamics
    volume_point_world = base_origin + base_rot @ sample.volume_pos
    force_point_world = base_origin + base_rot @ sample.force_pos
    environment = getattr(hyd, "water_environment_runtime", None)
    if environment is None:
        surface_height = float(runtime.water_surface_z)
    else:
        surface_height = environment.surface_height_world_m(
            volume_point_world,
            float(runtime.data.time),
        )
    point_depth = surface_height - float(volume_point_world[2])
    submerged = submerged_fraction(point_depth, sample.half_height, hyd.buoyancy_model)
    buoyancy_submerged = submerged_fraction(
        point_depth * hyd.buoyancy_slope_scale,
        sample.half_height,
        hyd.buoyancy_model,
    )
    buoyancy = runtime.rho * runtime.gravity * hyd.neutral_volume * sample.share * buoyancy_submerged * hyd.buoyancy_scale
    return np.array([0.0, 0.0, buoyancy], dtype=np.float64), force_point_world, submerged, buoyancy_submerged


def weighted_hydrostatic_result(
    weighted_submerged: float,
    weighted_buoyancy_submerged: float,
    weighted_point: np.ndarray,
    buoy_force_world: np.ndarray,
    buoy_tau_world: np.ndarray,
    cob: np.ndarray,
) -> HydrostaticWrenchResult:
    total_buoyancy = float(np.linalg.norm(buoy_force_world))
    if total_buoyancy > 1e-9:
        buoy_point = weighted_point / total_buoyancy
    else:
        buoy_point = cob.copy()
    return HydrostaticWrenchResult(
        submerged=float(np.clip(weighted_submerged, 0.0, 1.0)),
        buoyancy_submerged=float(np.clip(weighted_buoyancy_submerged, 0.0, 1.0)),
        buoy_force_world=buoy_force_world,
        buoy_tau_world=buoy_tau_world,
        buoy_point_world=buoy_point,
    )


__all__ = ["accumulate_weighted_hydrostatic", "weighted_hydrostatic_result"]
