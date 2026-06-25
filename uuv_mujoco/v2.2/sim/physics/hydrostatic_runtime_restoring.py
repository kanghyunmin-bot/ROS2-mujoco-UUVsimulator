"""Hydrostatic restoring runtime value extraction."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .hydrostatic_restoring_base import read_hydrostatic_restoring_base_values
from .hydrostatic_restoring_real_start import (
    read_real_start_restoring_trims,
    resolve_runtime_restoring_trims,
)


@dataclass(frozen=True)
class HydrostaticRestoringValues:
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


def read_hydrostatic_restoring_values(
    *,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
    real_start_required: bool,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
) -> HydrostaticRestoringValues:
    base = read_hydrostatic_restoring_base_values(
        hydro_cfg=hydro_cfg,
        sim_profile=sim_profile,
        env_float=env_float,
        env_flag=env_flag,
    )
    real_start_trims = read_real_start_restoring_trims(
        real_start_required=real_start_required,
        env_float=env_float,
    )
    trims = resolve_runtime_restoring_trims(
        hydro_cfg=hydro_cfg,
        real_start_required=real_start_required,
        env_flag=env_flag,
        real_start_trims=real_start_trims,
        roll_trim=base.roll_trim,
        pitch_trim=base.pitch_trim,
        profile_roll_trim=base.profile_roll_trim,
        profile_pitch_trim=base.profile_pitch_trim,
    )
    active = _restoring_active(base.active, base.roll_stiffness, base.pitch_stiffness)

    return HydrostaticRestoringValues(
        hydrostatic_restoring_active=active,
        hydrostatic_restoring_roll_stiffness=base.roll_stiffness,
        hydrostatic_restoring_pitch_stiffness=base.pitch_stiffness,
        hydrostatic_restoring_roll_trim_rad=trims.roll_trim,
        hydrostatic_restoring_pitch_trim_rad=trims.pitch_trim,
        hydrostatic_restoring_profile_roll_trim_rad=trims.profile_roll_trim,
        hydrostatic_restoring_profile_pitch_trim_rad=trims.profile_pitch_trim,
        hydrostatic_restoring_release_trim_blend_s=base.release_blend_s,
        real_start_restoring_roll_trim_rad=real_start_trims.roll_trim,
        real_start_restoring_pitch_trim_rad=real_start_trims.pitch_trim,
    )


def _restoring_active(active: bool, roll_stiffness: float, pitch_stiffness: float) -> bool:
    if roll_stiffness <= 0.0 and pitch_stiffness <= 0.0:
        return False
    return active


__all__ = ["HydrostaticRestoringValues", "read_hydrostatic_restoring_values"]
