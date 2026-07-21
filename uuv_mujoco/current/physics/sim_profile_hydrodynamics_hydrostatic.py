"""Hydrostatic profile part assembly for hydrodynamics config."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .sim_profile_parsing import parse_body_components, parse_buoyancy_points, parse_hydrostatic_restoring
from .sim_profile_types import BodyComponent, BuoyancyPoint


@dataclass(frozen=True)
class HydrostaticConfigParts:
    half_height: float
    body_components: tuple[BodyComponent, ...]
    buoyancy_points: tuple[BuoyancyPoint, ...]
    restoring_active: bool
    restoring_roll_stiffness: float
    restoring_pitch_stiffness: float
    restoring_roll_trim_rad: float
    restoring_pitch_trim_rad: float
    restoring_trim_from_real_start: bool


def build_hydrostatic_config_parts(
    sim_profile: Mapping[str, Any],
    *,
    ellipsoid_defaults: dict[str, Any] | None,
) -> HydrostaticConfigParts:
    half_height = default_half_height(sim_profile, ellipsoid_defaults)
    restoring = parse_hydrostatic_restoring(sim_profile)
    return HydrostaticConfigParts(
        half_height=half_height,
        body_components=parse_body_components(sim_profile),
        buoyancy_points=parse_buoyancy_points(sim_profile, half_height),
        restoring_active=restoring[0],
        restoring_roll_stiffness=restoring[1],
        restoring_pitch_stiffness=restoring[2],
        restoring_roll_trim_rad=restoring[3],
        restoring_pitch_trim_rad=restoring[4],
        restoring_trim_from_real_start=restoring[5],
    )


def default_half_height(sim_profile: Mapping[str, Any], ellipsoid_defaults: dict[str, Any] | None) -> float:
    return float(
        sim_profile.get(
            "half_height",
            ellipsoid_defaults["half_height"] if ellipsoid_defaults is not None else 0.147,
        )
    )


def thruster_force_limit(sim_profile: Mapping[str, Any], perf_force_max: float | None) -> float:
    thruster_force_max = float(sim_profile.get("thruster_force_max", 50.0))
    if perf_force_max is not None and float(perf_force_max) > 0.0:
        return float(perf_force_max)
    return thruster_force_max


__all__ = [
    "HydrostaticConfigParts",
    "build_hydrostatic_config_parts",
    "default_half_height",
    "thruster_force_limit",
]
