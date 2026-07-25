"""Field-group builders for hydrodynamics runtime values."""

from __future__ import annotations

import numpy as np


def full_heave_damping(hydro_cfg) -> float:
    return float(hydro_cfg.surface_heave_damping * hydro_cfg.heave_damping_scale)


def resolved_neutral_volume(hydro_cfg, neutral_volume: float) -> float:
    if hydro_cfg.displaced_volume is not None and hydro_cfg.displaced_volume > 0.0:
        return float(hydro_cfg.displaced_volume)
    return float(neutral_volume)


def scalar_hydrodynamics_fields(
    *,
    hydro_cfg,
    env_float,
    full_heave_damping_value: float,
    yaw_torque_scale_config: float,
    yaw_torque_scale: float,
    neutral_volume: float,
) -> dict[str, float | str]:
    return {
        "half_height": hydro_cfg.half_height,
        "buoyancy_model": hydro_cfg.buoyancy_model,
        "buoyancy_scale": hydro_cfg.buoyancy_scale,
        "buoyancy_slope_scale": hydro_cfg.buoyancy_slope_scale,
        "surface_heave_damping": hydro_cfg.surface_heave_damping,
        "heave_damping_scale": hydro_cfg.heave_damping_scale,
        "full_heave_damping": full_heave_damping_value,
        "cob_torque_scale": env_float("UUV_COB_TORQUE_SCALE", hydro_cfg.cob_torque_scale),
        "buoyancy_point_blend": hydro_cfg.buoyancy_point_blend,
        "thruster_force_max": hydro_cfg.thruster_force_max,
        "linear_drag": hydro_cfg.linear_drag,
        "angular_drag": hydro_cfg.angular_drag,
        "air_linear_drag": hydro_cfg.air_linear_drag,
        "air_angular_drag": hydro_cfg.air_angular_drag,
        "spin_gain": hydro_cfg.spin_gain,
        "yaw_torque_scale_config": yaw_torque_scale_config,
        "yaw_torque_scale": yaw_torque_scale,
        "neutral_volume": float(neutral_volume),
    }


def damping_array_fields(hydro_cfg) -> dict[str, np.ndarray]:
    return {
        "added_mass_diag": hydro_cfg.added_mass_diag.astype(np.float64, copy=True),
        "linear_damping_diag": hydro_cfg.linear_damping_diag.astype(np.float64, copy=True),
        "quadratic_damping_diag": hydro_cfg.quadratic_damping_diag.astype(np.float64, copy=True),
        "air_linear_damping_diag": hydro_cfg.air_linear_damping_diag.astype(np.float64, copy=True),
    }


def extra_coefficient_fields(
    *,
    hydro_pitch_moment_coeff: float,
    hydro_vertical_lift_coeff: float,
    hydro_vertical_lift_deadband_mps: float,
    hydro_vertical_lift_power: float,
    hydro_yawrate_heave_pos_coeff: float,
    hydro_yawrate_heave_neg_coeff: float,
    hydro_yawrate_heave_speed_deadband_mps: float,
    hydro_yawrate_heave_yaw_deadband_radps: float,
    heave_extra_damping_n_per_mps: float,
) -> dict[str, float]:
    return {
        "hydro_pitch_moment_coeff": hydro_pitch_moment_coeff,
        "hydro_vertical_lift_coeff": hydro_vertical_lift_coeff,
        "hydro_vertical_lift_deadband_mps": hydro_vertical_lift_deadband_mps,
        "hydro_vertical_lift_power": hydro_vertical_lift_power,
        "hydro_yawrate_heave_pos_coeff": hydro_yawrate_heave_pos_coeff,
        "hydro_yawrate_heave_neg_coeff": hydro_yawrate_heave_neg_coeff,
        "hydro_yawrate_heave_speed_deadband_mps": hydro_yawrate_heave_speed_deadband_mps,
        "hydro_yawrate_heave_yaw_deadband_radps": hydro_yawrate_heave_yaw_deadband_radps,
        "heave_extra_damping_n_per_mps": heave_extra_damping_n_per_mps,
    }


__all__ = [
    "damping_array_fields",
    "extra_coefficient_fields",
    "full_heave_damping",
    "resolved_neutral_volume",
    "scalar_hydrodynamics_fields",
]
