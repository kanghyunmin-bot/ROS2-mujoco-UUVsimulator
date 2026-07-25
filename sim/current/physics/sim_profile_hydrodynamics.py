"""Hydrodynamics config builder for simulation profiles."""

from __future__ import annotations

from typing import Any, Mapping

from .sim_profile_ellipsoid import resolve_ellipsoid_baseline
from .sim_profile_hydrodynamics_damping import build_damping_config
from .sim_profile_hydrodynamics_hydrostatic import build_hydrostatic_config_parts, thruster_force_limit
from .sim_profile_parsing import (
    clamp,
    normalize_buoyancy_model,
    normalize_hydrostatic_volume_source,
    vector3_from_keys,
)
from .sim_profile_types import HydrodynamicsConfig


def build_hydrodynamics_config(
    sim_profile: Mapping[str, Any],
    perf_force_max: float | None = None,
    fluid_density: float = 1000.0,
) -> HydrodynamicsConfig:
    """Group the main underwater dynamics knobs into one object."""

    linear_drag = float(sim_profile.get("linear_drag", 1.2))
    angular_drag = float(sim_profile.get("angular_drag", 0.12))
    ellipsoid_defaults, model_source = resolve_ellipsoid_baseline(sim_profile, fluid_density)
    damping = build_damping_config(
        sim_profile,
        ellipsoid_defaults=ellipsoid_defaults,
        linear_drag=linear_drag,
        angular_drag=angular_drag,
    )
    hydrostatic = build_hydrostatic_config_parts(
        sim_profile,
        ellipsoid_defaults=ellipsoid_defaults,
    )
    water_current_world = vector3_from_keys(
        sim_profile,
        "water_current_world",
        "current_world",
        default=(0.0, 0.0, 0.0),
    )

    return HydrodynamicsConfig(
        half_height=hydrostatic.half_height,
        buoyancy_model=normalize_buoyancy_model(sim_profile.get("buoyancy_model", "ellipsoid")),
        buoyancy_scale=float(sim_profile.get("buoyancy_scale", 1.0)),
        buoyancy_slope_scale=max(float(sim_profile.get("buoyancy_slope_scale", 1.0)), 0.1),
        surface_heave_damping=max(float(sim_profile.get("surface_heave_damping", 0.0)), 0.0),
        heave_damping_scale=max(float(sim_profile.get("heave_damping_scale", 1.0)), 0.0),
        cob_torque_scale=float(sim_profile.get("cob_torque_scale", 1.0)),
        buoyancy_point_blend=clamp(float(sim_profile.get("buoyancy_point_blend", 1.0)), 0.0, 1.0),
        thruster_force_max=thruster_force_limit(sim_profile, perf_force_max),
        linear_drag=linear_drag,
        angular_drag=angular_drag,
        air_linear_drag=float(damping.air_linear_damping_diag[0]),
        air_angular_drag=float(damping.air_linear_damping_diag[3]),
        spin_gain=float(sim_profile.get("spin_gain", 22.0)),
        yaw_torque_scale=float(sim_profile.get("yaw_torque_scale", 1.0)),
        added_mass_diag=damping.added_mass_diag,
        linear_damping_diag=damping.linear_damping_diag,
        quadratic_damping_diag=damping.quadratic_damping_diag,
        air_linear_damping_diag=damping.air_linear_damping_diag,
        water_current_world=water_current_world,
        displaced_volume=(
            None if ellipsoid_defaults is None else ellipsoid_defaults["displaced_volume"]
        ),
        model_source=model_source,
        ellipsoid_semi_axes=(
            None if ellipsoid_defaults is None else ellipsoid_defaults["semi_axes"].copy()
        ),
        body_components=hydrostatic.body_components,
        buoyancy_points=hydrostatic.buoyancy_points,
        hydrostatic_volume_source=normalize_hydrostatic_volume_source(
            sim_profile.get("hydrostatic_volume_source", "auto")
        ),
        hydrostatic_restoring_active=hydrostatic.restoring_active,
        hydrostatic_restoring_roll_stiffness=hydrostatic.restoring_roll_stiffness,
        hydrostatic_restoring_pitch_stiffness=hydrostatic.restoring_pitch_stiffness,
        hydrostatic_restoring_roll_trim_rad=hydrostatic.restoring_roll_trim_rad,
        hydrostatic_restoring_pitch_trim_rad=hydrostatic.restoring_pitch_trim_rad,
        hydrostatic_restoring_trim_from_real_start=hydrostatic.restoring_trim_from_real_start,
    )


__all__ = ["build_hydrodynamics_config"]
