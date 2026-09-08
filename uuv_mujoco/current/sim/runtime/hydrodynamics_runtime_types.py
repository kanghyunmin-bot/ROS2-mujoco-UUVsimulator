"""Typed containers for hydrodynamics runtime setup."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass
class HydrodynamicsRuntimeValues:
    half_height: float
    buoyancy_model: str
    buoyancy_scale: float
    buoyancy_slope_scale: float
    surface_heave_damping: float
    heave_damping_scale: float
    full_heave_damping: float
    cob_torque_scale: float
    buoyancy_point_blend: float
    thruster_force_max: float
    linear_drag: float
    angular_drag: float
    air_linear_drag: float
    air_angular_drag: float
    spin_gain: float
    yaw_torque_scale_config: float
    yaw_torque_scale: float
    yaw_torque_thruster_scales: dict[str, float]
    water_current_world: Any
    added_mass_diag: Any
    linear_damping_diag: Any
    quadratic_damping_diag: Any
    air_linear_damping_diag: Any
    body_components: list
    buoyancy_points: list
    hydro_pitch_moment_coeff: float
    hydro_vertical_lift_coeff: float
    hydro_vertical_lift_deadband_mps: float
    hydro_vertical_lift_power: float
    hydro_yawrate_heave_pos_coeff: float
    hydro_yawrate_heave_neg_coeff: float
    hydro_yawrate_heave_speed_deadband_mps: float
    hydro_yawrate_heave_yaw_deadband_radps: float
    heave_extra_damping_n_per_mps: float
    neutral_volume: float


@dataclass
class HydrodynamicWrenchRuntime:
    cfd_dynamic_wrench: Any
    cfd_dynamic_wrench_enabled: bool
    cfd_dynamic_wrench_scale: float
    cfd_dynamic_wrench_debug: bool
    cfd_dynamic_wrench_last_log_sim_t: dict[str, float]
    cfd_dynamic_wrench_axes: Any
    cfd_dynamic_wrench_owns_z: bool
    residual_hydro: Any
    residual_hydro_active: bool
    residual_hydro_coeffs: Any
    fossen_residual: Any
    fossen_residual_active: bool
    fossen_residual_added_mass_active: bool
    fossen_residual_requested_active: bool
    fossen_residual_requested_added_mass_active: bool
    fossen_residual_linear: Any
    fossen_residual_forward_speed: float
    fossen_residual_quadratic: Any
    fossen_residual_added_mass_matrix: Any


@dataclass
class HydrodynamicsRuntimeSetup:
    half_height: float
    buoyancy_model: str
    buoyancy_scale: float
    buoyancy_slope_scale: float
    surface_heave_damping: float
    heave_damping_scale: float
    full_heave_damping: float
    cob_torque_scale: float
    buoyancy_point_blend: float
    thruster_force_max: float
    linear_drag: float
    angular_drag: float
    air_linear_drag: float
    air_angular_drag: float
    spin_gain: float
    yaw_torque_scale_config: float
    yaw_torque_scale: float
    yaw_torque_thruster_scales: dict[str, float]
    water_current_world: Any
    added_mass_diag: Any
    linear_damping_diag: Any
    quadratic_damping_diag: Any
    air_linear_damping_diag: Any
    body_components: list
    buoyancy_points: list
    hydro_pitch_moment_coeff: float
    hydro_vertical_lift_coeff: float
    hydro_vertical_lift_deadband_mps: float
    hydro_vertical_lift_power: float
    hydro_yawrate_heave_pos_coeff: float
    hydro_yawrate_heave_neg_coeff: float
    hydro_yawrate_heave_speed_deadband_mps: float
    hydro_yawrate_heave_yaw_deadband_radps: float
    heave_extra_damping_n_per_mps: float
    cfd_dynamic_wrench: Any
    cfd_dynamic_wrench_enabled: bool
    cfd_dynamic_wrench_scale: float
    cfd_dynamic_wrench_debug: bool
    cfd_dynamic_wrench_last_log_sim_t: dict[str, float]
    cfd_dynamic_wrench_axes: Any
    cfd_dynamic_wrench_owns_z: bool
    residual_hydro: Any
    residual_hydro_active: bool
    residual_hydro_coeffs: Any
    fossen_residual: Any
    fossen_residual_active: bool
    fossen_residual_added_mass_active: bool
    fossen_residual_requested_active: bool
    fossen_residual_requested_added_mass_active: bool
    fossen_residual_linear: Any
    fossen_residual_forward_speed: float
    fossen_residual_quadratic: Any
    fossen_residual_added_mass_matrix: Any
    neutral_volume: float
    thruster_loop_hz: float
    thruster_loop_dt: float
    thruster_scheduler: Any
    dynamic_fluidcoef_runtime: Any
    current_field_runtime: Any
    state_coefficient_scaler: Any
    state_fluidcoef_runtime: Any
    free_surface: Any
    water_environment_runtime: Any
    distributed_hydrodynamics: Any
    full_matrix_hydrodynamics: Any
    update_dynamic_fluidcoef: Callable


__all__ = [
    "HydrodynamicWrenchRuntime",
    "HydrodynamicsRuntimeSetup",
    "HydrodynamicsRuntimeValues",
]
