"""Initial-depth runtime construction for the MuJoCo runner."""

from __future__ import annotations

from physics.thruster_mapping import PHYSICAL_VERTICAL_THRUSTERS, PHYSICAL_YAW_THRUSTERS
from sim.runtime.initial_depth_runtime import InitialDepthHoldRuntime
from sim.runtime.initial_state import configure_initial_runtime_state


def create_initial_depth_runtime_setup(
    *,
    args,
    mujoco_module,
    model_setup,
    sim_profile: dict,
    thruster_immersion_half_height_m: float,
    env_float,
    env_flag,
) -> tuple[object, InitialDepthHoldRuntime]:
    base_state = model_setup.base_state
    initial_runtime_state = configure_initial_runtime_state(
        args=args,
        mujoco=mujoco_module,
        model=model_setup.model,
        data=model_setup.data,
        sim_profile=sim_profile,
        base_state=base_state,
        fluid_geom_ids=model_setup.fluid_geom_ids,
        fluid_geom_names=model_setup.fluid_geom_names,
        thruster_names=PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS,
        thruster_immersion_half_height_m=thruster_immersion_half_height_m,
        env_float=env_float,
        env_flag=env_flag,
    )
    initial_depth_runtime = InitialDepthHoldRuntime(
        state=initial_runtime_state.initial_depth_hold,
        data=model_setup.data,
        mujoco=mujoco_module,
        model=model_setup.model,
        base_id=base_state.base_id,
        world_qpos_adr=base_state.world_qpos_adr,
        world_qvel_adr=base_state.world_qvel_adr,
        set_bar30_depth=base_state.set_bar30_depth,
        set_base_depth=base_state.set_base_depth,
    )
    if initial_runtime_state.real_start_required and not initial_runtime_state.initial_depth_hold["active"]:
        initial_depth_runtime.apply_release_velocity_state()
    return initial_runtime_state, initial_depth_runtime


__all__ = ["create_initial_depth_runtime_setup"]
