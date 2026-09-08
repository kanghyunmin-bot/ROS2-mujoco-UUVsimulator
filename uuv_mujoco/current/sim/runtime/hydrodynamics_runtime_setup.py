"""Hydrodynamics runtime setup and diagnostics for the main MuJoCo loop."""

from __future__ import annotations

from typing import Callable

import numpy as np

from sim.physics.current_field import DeterministicCurrentField
from sim.physics.current_field_runtime import CurrentFieldRuntime
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics
from sim.physics.dynamic_fluidcoef import DynamicFluidcoefRuntime
from sim.physics.fluidcoef_immersion_runtime import FluidcoefImmersionRuntime
from sim.physics.free_surface import FreeSurface
from sim.physics.full_matrix_hydrodynamics import FullMatrixHydrodynamics
from sim.physics.hydrodynamic_state_scaling import (
    FluidcoefStateScalingRuntime,
    HydrodynamicStateScaler,
)
from sim.runtime.fixed_rate_scheduler import FixedRateSimScheduler
from sim.runtime.hydrodynamics_runtime_ownership import (
    validate_advanced_hydrodynamics_ownership,
)
from sim.runtime.hydrodynamics_runtime_logging import log_hydrodynamics_runtime_setup
from sim.runtime.hydrodynamics_runtime_types import HydrodynamicsRuntimeSetup
from sim.runtime.hydrodynamics_runtime_values import collect_hydrodynamics_runtime_values
from sim.runtime.hydrodynamics_runtime_wrenches import build_hydrodynamic_wrench_runtime
from sim.runtime.water_environment_runtime import WaterEnvironmentRuntime


def build_hydrodynamics_runtime_setup(
    *,
    args,
    model,
    data,
    mujoco_module,
    sim_profile: dict,
    hydro_cfg,
    use_custom_hydrodynamics: bool,
    active_body_components: list,
    active_buoyancy_points: list,
    fluidcoef_dynamic_setup,
    fluid_geom_names: dict[int, str],
    neutral_volume: float,
    vehicle_mass: float,
    rho: float,
    water_surface_z: float,
    thruster_air_force_scale: float,
    thruster_immersion_half_height_m: float,
    env_float,
    env_flag,
    to_float_array,
    log: Callable[[str], None],
) -> HydrodynamicsRuntimeSetup:
    """Build non-loop hydrodynamic runtime state and emit legacy diagnostics."""

    values = collect_hydrodynamics_runtime_values(
        model=model,
        sim_profile=sim_profile,
        hydro_cfg=hydro_cfg,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        active_body_components=active_body_components,
        active_buoyancy_points=active_buoyancy_points,
        neutral_volume=neutral_volume,
        env_float=env_float,
        log=log,
    )
    wrenches = build_hydrodynamic_wrench_runtime(
        sim_profile=sim_profile,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        heave_extra_damping_n_per_mps=values.heave_extra_damping_n_per_mps,
        env_float=env_float,
        env_flag=env_flag,
        to_float_array=to_float_array,
        log=log,
    )
    thruster_loop_hz = float(np.clip(args.thruster_loop_hz, 1.0, 500.0))
    thruster_loop_dt = 1.0 / thruster_loop_hz
    dynamic_fluidcoef_runtime = DynamicFluidcoefRuntime(
        model=model,
        data=data,
        mujoco_module=mujoco_module,
        setup=fluidcoef_dynamic_setup,
        water_current_world=values.water_current_world,
        fluid_geom_names=fluid_geom_names,
        env_float=env_float,
        env_flag=env_flag,
        to_float_array=to_float_array,
    )
    fluidcoef_immersion_runtime = FluidcoefImmersionRuntime(
        model=model,
        data=data,
        fluid_geom_ids=fluid_geom_names.keys(),
        water_surface_z=water_surface_z,
        enabled=(
            not use_custom_hydrodynamics
            and bool(sim_profile.get("mujoco_fluidcoef_immersion_scale", True))
        ),
        update_unscaled=dynamic_fluidcoef_runtime.update,
    )
    state_coefficient_scaler = HydrodynamicStateScaler.from_profile(sim_profile)
    state_fluidcoef_runtime = FluidcoefStateScalingRuntime(
        model=model,
        fluid_geom_ids=fluid_geom_names.keys(),
        enabled=(not use_custom_hydrodynamics and state_coefficient_scaler.active),
        update_unscaled=fluidcoef_immersion_runtime.update,
    )
    current_field = DeterministicCurrentField.from_profile(
        sim_profile,
        fallback_velocity_world_mps=values.water_current_world,
    )
    current_field_runtime = CurrentFieldRuntime(
        model=model,
        field=current_field,
        water_current_world=values.water_current_world,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        log=log,
    )
    free_surface = FreeSurface.from_profile(
        sim_profile,
        reference_height_world_m=float(water_surface_z),
    )
    water_environment_runtime = WaterEnvironmentRuntime(
        current_field=current_field,
        free_surface=free_surface,
        fallback_surface_height_world_m=float(water_surface_z),
    )
    fluidcoef_immersion_runtime.set_surface_height_sampler(
        water_environment_runtime.surface_height_world_m
    )
    distributed_hydrodynamics = DistributedHullHydrodynamics.from_profile(sim_profile)
    full_matrix_hydrodynamics = FullMatrixHydrodynamics.from_profile(sim_profile)
    validate_advanced_hydrodynamics_ownership(
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        hydro_cfg=hydro_cfg,
        values=values,
        wrenches=wrenches,
        state_coefficient_scaler=state_coefficient_scaler,
        distributed_hydrodynamics=distributed_hydrodynamics,
        full_matrix_hydrodynamics=full_matrix_hydrodynamics,
        neutral_volume_m3=values.neutral_volume,
        rho_kg_m3=rho,
        gravity_world_mps2=model.opt.gravity,
    )
    log(
        "[physics] MuJoCo fluidcoef waterline immersion scaling: "
        f"{'on' if fluidcoef_immersion_runtime.enabled else 'off'} "
        f"(geoms={int(fluidcoef_immersion_runtime.geom_ids.size)})"
    )
    log(
        "[physics] spatial current field: "
        f"{'on' if current_field.active else 'off'} "
        f"(status={current_field.config.calibration_status})"
    )
    log(
        "[physics] speed/depth/tilt coefficient scaling: "
        f"{'on' if state_coefficient_scaler.active else 'off'} "
        f"(status={state_coefficient_scaler.calibration_status})"
    )
    log(
        "[physics] shared free surface: "
        f"{'on' if free_surface.active else 'off'} "
        f"(mode={free_surface.mode}, status={free_surface.config.calibration_status})"
    )
    log(
        "[physics] distributed hull patches: "
        f"{'on' if distributed_hydrodynamics.active else 'off'} "
        f"(patches={distributed_hydrodynamics.config.patch_count}, "
        f"status={distributed_hydrodynamics.config.calibration_status})"
    )
    log(
        "[physics] full 6x6 hydrodynamic matrices: "
        f"{'on' if full_matrix_hydrodynamics.active else 'off'} "
        f"(status={full_matrix_hydrodynamics.config.calibration_status})"
    )
    log_hydrodynamics_runtime_setup(
        values=values,
        hydro_cfg=hydro_cfg,
        model=model,
        vehicle_mass=vehicle_mass,
        rho=rho,
        water_surface_z=water_surface_z,
        thruster_air_force_scale=thruster_air_force_scale,
        thruster_immersion_half_height_m=thruster_immersion_half_height_m,
        thruster_loop_hz=thruster_loop_hz,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        log=log,
    )
    return HydrodynamicsRuntimeSetup(
        **vars(values),
        **vars(wrenches),
        thruster_loop_hz=thruster_loop_hz,
        thruster_loop_dt=thruster_loop_dt,
        thruster_scheduler=FixedRateSimScheduler(dt=thruster_loop_dt),
        dynamic_fluidcoef_runtime=dynamic_fluidcoef_runtime,
        current_field_runtime=current_field_runtime,
        state_coefficient_scaler=state_coefficient_scaler,
        state_fluidcoef_runtime=state_fluidcoef_runtime,
        free_surface=free_surface,
        water_environment_runtime=water_environment_runtime,
        distributed_hydrodynamics=distributed_hydrodynamics,
        full_matrix_hydrodynamics=full_matrix_hydrodynamics,
        update_dynamic_fluidcoef=state_fluidcoef_runtime.update,
    )


__all__ = ["HydrodynamicsRuntimeSetup", "build_hydrodynamics_runtime_setup"]
