"""Hydrodynamics runtime setup and diagnostics for the main MuJoCo loop."""

from __future__ import annotations

from typing import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef import DynamicFluidcoefRuntime
from sim.runtime.fixed_rate_scheduler import FixedRateSimScheduler
from sim.runtime.hydrodynamics_runtime_logging import log_hydrodynamics_runtime_setup
from sim.runtime.hydrodynamics_runtime_types import HydrodynamicsRuntimeSetup
from sim.runtime.hydrodynamics_runtime_values import collect_hydrodynamics_runtime_values
from sim.runtime.hydrodynamics_runtime_wrenches import build_hydrodynamic_wrench_runtime


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
        update_dynamic_fluidcoef=dynamic_fluidcoef_runtime.update,
    )


__all__ = ["HydrodynamicsRuntimeSetup", "build_hydrodynamics_runtime_setup"]
