"""Scalar and array value extraction for hydrodynamics runtime setup."""

from __future__ import annotations

from typing import Callable

import numpy as np

from sim.runtime.hydrodynamics_runtime_coefficients import (
    log_extra_hydro_coefficients,
    log_yaw_scale_override,
    log_yaw_torque_thruster_scales,
    read_extra_hydro_coefficients,
    read_yaw_torque_scale,
    read_yaw_torque_thruster_scales,
)
from sim.runtime.hydrodynamics_runtime_current import configure_mujoco_current
from sim.runtime.hydrodynamics_runtime_types import HydrodynamicsRuntimeValues
from sim.runtime.hydrodynamics_runtime_value_fields import (
    damping_array_fields,
    extra_coefficient_fields,
    full_heave_damping,
    resolved_neutral_volume,
    scalar_hydrodynamics_fields,
)


def collect_hydrodynamics_runtime_values(
    *,
    model,
    sim_profile: dict,
    hydro_cfg,
    use_custom_hydrodynamics: bool,
    active_body_components: list,
    active_buoyancy_points: list,
    neutral_volume: float,
    env_float,
    log: Callable[[str], None],
) -> HydrodynamicsRuntimeValues:
    """Collect hydrodynamic scalars/arrays without constructing runtime objects."""

    full_heave_damping_value = full_heave_damping(hydro_cfg)
    yaw_torque_scale_config, yaw_torque_scale = read_yaw_torque_scale(
        hydro_cfg=hydro_cfg,
        env_float=env_float,
    )
    log_yaw_scale_override(yaw_torque_scale, log)
    yaw_torque_thruster_scales = read_yaw_torque_thruster_scales(
        sim_profile=sim_profile,
        env_float=env_float,
    )
    log_yaw_torque_thruster_scales(yaw_torque_thruster_scales, log)

    water_current_world = hydro_cfg.water_current_world.astype(np.float64, copy=True)
    configure_mujoco_current(
        model,
        water_current_world=water_current_world,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        log=log,
    )
    (
        hydro_pitch_moment_coeff,
        hydro_vertical_lift_coeff,
        hydro_vertical_lift_deadband_mps,
        hydro_vertical_lift_power,
        hydro_yawrate_heave_pos_coeff,
        hydro_yawrate_heave_neg_coeff,
        hydro_yawrate_heave_speed_deadband_mps,
        hydro_yawrate_heave_yaw_deadband_radps,
        heave_extra_damping_n_per_mps,
    ) = read_extra_hydro_coefficients(
        sim_profile=sim_profile,
        env_float=env_float,
    )
    log_extra_hydro_coefficients(
        hydro_pitch_moment_coeff=hydro_pitch_moment_coeff,
        hydro_vertical_lift_coeff=hydro_vertical_lift_coeff,
        hydro_vertical_lift_deadband_mps=hydro_vertical_lift_deadband_mps,
        hydro_vertical_lift_power=hydro_vertical_lift_power,
        hydro_yawrate_heave_pos_coeff=hydro_yawrate_heave_pos_coeff,
        hydro_yawrate_heave_neg_coeff=hydro_yawrate_heave_neg_coeff,
        hydro_yawrate_heave_speed_deadband_mps=hydro_yawrate_heave_speed_deadband_mps,
        hydro_yawrate_heave_yaw_deadband_radps=hydro_yawrate_heave_yaw_deadband_radps,
        heave_extra_damping_n_per_mps=heave_extra_damping_n_per_mps,
        log=log,
    )

    return HydrodynamicsRuntimeValues(
        **scalar_hydrodynamics_fields(
            hydro_cfg=hydro_cfg,
            env_float=env_float,
            full_heave_damping_value=full_heave_damping_value,
            yaw_torque_scale_config=yaw_torque_scale_config,
            yaw_torque_scale=yaw_torque_scale,
            neutral_volume=resolved_neutral_volume(hydro_cfg, neutral_volume),
        ),
        yaw_torque_thruster_scales=yaw_torque_thruster_scales,
        water_current_world=water_current_world,
        **damping_array_fields(hydro_cfg),
        body_components=active_body_components,
        buoyancy_points=active_buoyancy_points,
        **extra_coefficient_fields(
            hydro_pitch_moment_coeff=hydro_pitch_moment_coeff,
            hydro_vertical_lift_coeff=hydro_vertical_lift_coeff,
            hydro_vertical_lift_deadband_mps=hydro_vertical_lift_deadband_mps,
            hydro_vertical_lift_power=hydro_vertical_lift_power,
            hydro_yawrate_heave_pos_coeff=hydro_yawrate_heave_pos_coeff,
            hydro_yawrate_heave_neg_coeff=hydro_yawrate_heave_neg_coeff,
            hydro_yawrate_heave_speed_deadband_mps=hydro_yawrate_heave_speed_deadband_mps,
            hydro_yawrate_heave_yaw_deadband_radps=hydro_yawrate_heave_yaw_deadband_radps,
            heave_extra_damping_n_per_mps=heave_extra_damping_n_per_mps,
        ),
    )
__all__ = [
    "collect_hydrodynamics_runtime_values",
    "configure_mujoco_current",
    "log_extra_hydro_coefficients",
    "log_yaw_scale_override",
]
