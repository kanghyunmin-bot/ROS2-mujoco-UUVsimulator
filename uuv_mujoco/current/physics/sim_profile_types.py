"""Typed simulation profile outputs for hydrodynamics setup."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HydrodynamicsConfig:
    """Physics knobs that are commonly tuned together for underwater feel."""

    half_height: float
    buoyancy_model: str
    buoyancy_scale: float
    buoyancy_slope_scale: float
    surface_heave_damping: float
    heave_damping_scale: float
    cob_torque_scale: float
    buoyancy_point_blend: float
    thruster_force_max: float
    linear_drag: float
    angular_drag: float
    air_linear_drag: float
    air_angular_drag: float
    spin_gain: float
    yaw_torque_scale: float
    added_mass_diag: np.ndarray
    linear_damping_diag: np.ndarray
    quadratic_damping_diag: np.ndarray
    air_linear_damping_diag: np.ndarray
    water_current_world: np.ndarray
    displaced_volume: float | None
    model_source: str
    ellipsoid_semi_axes: np.ndarray | None
    body_components: tuple["BodyComponent", ...]
    buoyancy_points: tuple["BuoyancyPoint", ...]
    hydrostatic_volume_source: str
    hydrostatic_restoring_active: bool
    hydrostatic_restoring_roll_stiffness: float
    hydrostatic_restoring_pitch_stiffness: float
    hydrostatic_restoring_roll_trim_rad: float
    hydrostatic_restoring_pitch_trim_rad: float
    hydrostatic_restoring_trim_from_real_start: bool


@dataclass(frozen=True)
class BodyComponent:
    """Intuitive component-level mass/buoyancy definition for one rigid body."""

    name: str
    shape: str
    size: np.ndarray
    mass: float
    mass_pos: np.ndarray
    buoyancy_pos: np.ndarray
    buoyancy_share: float


@dataclass(frozen=True)
class BuoyancyPoint:
    """Distributed buoyancy application point."""

    name: str
    pos: np.ndarray
    share: float
    half_height: float


__all__ = ["HydrodynamicsConfig", "BodyComponent", "BuoyancyPoint"]
