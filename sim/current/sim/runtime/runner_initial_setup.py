"""Model loading and initial-state setup for the MuJoCo runner."""

from __future__ import annotations

from sim.runtime.model_runtime_setup import load_model_runtime_setup
from sim.runtime.runner_initial_depth_setup import create_initial_depth_runtime_setup
from sim.runtime.runner_initial_setup_types import RunnerInitialSetup
from sim.runtime.runner_initial_thruster_contract import resolve_thruster_immersion_contract


def load_runner_initial_setup(
    *,
    args,
    mujoco_module,
    sim_profile: dict,
    uuv_run_mode: str,
    env_float,
    env_flag,
    env_get,
    to_float_array,
    to_float_matrix,
) -> RunnerInitialSetup:
    model_setup = load_model_runtime_setup(
        args=args,
        mujoco_module=mujoco_module,
        sim_profile=sim_profile,
        run_mode=uuv_run_mode,
        env_float=env_float,
        env_flag=env_flag,
        env_get=env_get,
        to_float_array=to_float_array,
        to_float_matrix=to_float_matrix,
    )
    base_state = model_setup.base_state
    thruster_air_force_scale, thruster_immersion_half_height_m = resolve_thruster_immersion_contract(env_float)
    initial_runtime_state, initial_depth_runtime = create_initial_depth_runtime_setup(
        args=args,
        mujoco_module=mujoco_module,
        model_setup=model_setup,
        sim_profile=sim_profile,
        thruster_immersion_half_height_m=thruster_immersion_half_height_m,
        env_float=env_float,
        env_flag=env_flag,
    )

    return RunnerInitialSetup(
        model_setup=model_setup,
        model=model_setup.model,
        data=model_setup.data,
        scene_fluid_density=float(model_setup.scene_fluid_density),
        scene_fluid_viscosity=float(model_setup.scene_fluid_viscosity),
        fluid_geom_ids=list(model_setup.fluid_geom_ids),
        fluid_geom_names=dict(model_setup.fluid_geom_names),
        fluidcoef_dynamic_setup=model_setup.fluidcoef_dynamic_setup,
        base_state=base_state,
        base_id=base_state.base_id,
        world_qpos_adr=base_state.world_qpos_adr,
        world_qvel_adr=base_state.world_qvel_adr,
        water_surface_z=base_state.water_surface_z,
        base_origin_world=base_state.base_origin_world,
        set_base_depth=base_state.set_base_depth,
        bar30_depth_now_m=base_state.bar30_depth_now_m,
        set_bar30_depth=base_state.set_bar30_depth,
        thruster_air_force_scale=thruster_air_force_scale,
        thruster_immersion_half_height_m=thruster_immersion_half_height_m,
        initial_runtime_state=initial_runtime_state,
        initial_depth_hold=initial_runtime_state.initial_depth_hold,
        initial_depth_hold_auto_release=initial_runtime_state.initial_depth_hold_auto_release,
        real_start_required=bool(initial_runtime_state.real_start_required),
        initial_depth_runtime=initial_depth_runtime,
        apply_initial_depth_hold=initial_depth_runtime.apply_hold,
        apply_release_velocity_state=initial_depth_runtime.apply_release_velocity_state,
    )


__all__ = ["RunnerInitialSetup", "load_runner_initial_setup"]
