"""Hydrostatic setup facade for the MuJoCo UUV runtime."""

from __future__ import annotations

from typing import Any, Callable

from .hydrostatic_runtime_reporting import (
    align_cob_site,
    log_cob_runtime_override,
    log_hydrostatic_runtime,
)
from .hydrostatic_runtime_sources import resolve_hydrostatic_sources
from .hydrostatic_runtime_types import HydrostaticRuntimeConfig
from .hydrostatic_runtime_values import read_hydrostatic_runtime_values


def configure_hydrostatic_runtime(
    *,
    model: Any,
    mujoco_module: Any,
    base_id: int,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
    real_start_required: bool,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
    log: Callable[[str], None],
) -> HydrostaticRuntimeConfig:
    cob_site_id = int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_SITE, "cob_site"))
    values = read_hydrostatic_runtime_values(
        hydro_cfg=hydro_cfg,
        sim_profile=sim_profile,
        real_start_required=real_start_required,
        env_float=env_float,
        env_flag=env_flag,
    )
    active_body_components, active_buoyancy_points, hydrostatic_source_used = resolve_hydrostatic_sources(hydro_cfg)
    log_cob_runtime_override(values=values, hydro_cfg=hydro_cfg, log=log)
    align_cob_site(model=model, base_id=base_id, cob_site_id=cob_site_id, values=values, log=log)
    log_hydrostatic_runtime(
        values=values,
        active_body_components=active_body_components,
        active_buoyancy_points=active_buoyancy_points,
        hydrostatic_source_used=hydrostatic_source_used,
        real_start_required=real_start_required,
        log=log,
    )

    return HydrostaticRuntimeConfig(
        cob_site_id=cob_site_id,
        cob_torque_scale=values.cob_torque_scale,
        cob_longitudinal_offset=values.cob_longitudinal_offset,
        cob_vertical_offset=values.cob_vertical_offset,
        active_body_components=active_body_components,
        active_buoyancy_points=active_buoyancy_points,
        hydrostatic_source_used=hydrostatic_source_used,
        hydrostatic_restoring_active=values.hydrostatic_restoring_active,
        hydrostatic_restoring_roll_stiffness=values.hydrostatic_restoring_roll_stiffness,
        hydrostatic_restoring_pitch_stiffness=values.hydrostatic_restoring_pitch_stiffness,
        hydrostatic_restoring_roll_trim_rad=values.hydrostatic_restoring_roll_trim_rad,
        hydrostatic_restoring_pitch_trim_rad=values.hydrostatic_restoring_pitch_trim_rad,
        hydrostatic_restoring_profile_roll_trim_rad=values.hydrostatic_restoring_profile_roll_trim_rad,
        hydrostatic_restoring_profile_pitch_trim_rad=values.hydrostatic_restoring_profile_pitch_trim_rad,
        hydrostatic_restoring_release_trim_blend_s=values.hydrostatic_restoring_release_trim_blend_s,
        real_start_restoring_roll_trim_rad=values.real_start_restoring_roll_trim_rad,
        real_start_restoring_pitch_trim_rad=values.real_start_restoring_pitch_trim_rad,
    )


__all__ = ["HydrostaticRuntimeConfig", "configure_hydrostatic_runtime"]
