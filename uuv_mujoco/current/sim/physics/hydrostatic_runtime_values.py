"""Hydrostatic runtime env/profile value extraction facade."""

from __future__ import annotations

from typing import Any, Callable

from .hydrostatic_runtime_cob import read_hydrostatic_cob_values
from .hydrostatic_runtime_restoring import read_hydrostatic_restoring_values
from .hydrostatic_runtime_types import HydrostaticRuntimeValues


def read_hydrostatic_runtime_values(
    *,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
    real_start_required: bool,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
) -> HydrostaticRuntimeValues:
    cob_values = read_hydrostatic_cob_values(
        hydro_cfg=hydro_cfg,
        sim_profile=sim_profile,
        env_float=env_float,
    )
    restoring_values = read_hydrostatic_restoring_values(
        hydro_cfg=hydro_cfg,
        sim_profile=sim_profile,
        real_start_required=real_start_required,
        env_float=env_float,
        env_flag=env_flag,
    )
    return HydrostaticRuntimeValues(
        profile_cob_x_offset=cob_values.profile_cob_x_offset,
        profile_cob_z_offset=cob_values.profile_cob_z_offset,
        cob_torque_scale=cob_values.cob_torque_scale,
        cob_longitudinal_offset=cob_values.cob_longitudinal_offset,
        cob_vertical_offset=cob_values.cob_vertical_offset,
        hydrostatic_restoring_active=restoring_values.hydrostatic_restoring_active,
        hydrostatic_restoring_roll_stiffness=restoring_values.hydrostatic_restoring_roll_stiffness,
        hydrostatic_restoring_pitch_stiffness=restoring_values.hydrostatic_restoring_pitch_stiffness,
        hydrostatic_restoring_roll_trim_rad=restoring_values.hydrostatic_restoring_roll_trim_rad,
        hydrostatic_restoring_pitch_trim_rad=restoring_values.hydrostatic_restoring_pitch_trim_rad,
        hydrostatic_restoring_profile_roll_trim_rad=restoring_values.hydrostatic_restoring_profile_roll_trim_rad,
        hydrostatic_restoring_profile_pitch_trim_rad=restoring_values.hydrostatic_restoring_profile_pitch_trim_rad,
        hydrostatic_restoring_release_trim_blend_s=restoring_values.hydrostatic_restoring_release_trim_blend_s,
        real_start_restoring_roll_trim_rad=restoring_values.real_start_restoring_roll_trim_rad,
        real_start_restoring_pitch_trim_rad=restoring_values.real_start_restoring_pitch_trim_rad,
    )


__all__ = ["read_hydrostatic_runtime_values"]
