"""Hydrostatic runtime data records."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class HydrostaticRuntimeValues:
    profile_cob_x_offset: float
    profile_cob_z_offset: float
    cob_torque_scale: float
    cob_longitudinal_offset: float
    cob_vertical_offset: float
    hydrostatic_restoring_active: bool
    hydrostatic_restoring_roll_stiffness: float
    hydrostatic_restoring_pitch_stiffness: float
    hydrostatic_restoring_roll_trim_rad: float
    hydrostatic_restoring_pitch_trim_rad: float
    hydrostatic_restoring_profile_roll_trim_rad: float
    hydrostatic_restoring_profile_pitch_trim_rad: float
    hydrostatic_restoring_release_trim_blend_s: float
    real_start_restoring_roll_trim_rad: float
    real_start_restoring_pitch_trim_rad: float


@dataclass(frozen=True)
class HydrostaticRuntimeConfig:
    cob_site_id: int
    cob_torque_scale: float
    cob_longitudinal_offset: float
    cob_vertical_offset: float
    active_body_components: tuple[Any, ...]
    active_buoyancy_points: tuple[Any, ...]
    hydrostatic_source_used: str
    hydrostatic_restoring_active: bool
    hydrostatic_restoring_roll_stiffness: float
    hydrostatic_restoring_pitch_stiffness: float
    hydrostatic_restoring_roll_trim_rad: float
    hydrostatic_restoring_pitch_trim_rad: float
    hydrostatic_restoring_profile_roll_trim_rad: float
    hydrostatic_restoring_profile_pitch_trim_rad: float
    hydrostatic_restoring_release_trim_blend_s: float
    real_start_restoring_roll_trim_rad: float
    real_start_restoring_pitch_trim_rad: float


__all__ = ["HydrostaticRuntimeValues", "HydrostaticRuntimeConfig"]
