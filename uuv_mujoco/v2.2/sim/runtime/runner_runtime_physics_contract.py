"""Runner adapter for runtime physics setup."""

from __future__ import annotations

from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
    PHYSICAL_VERTICAL_THRUSTERS,
    PHYSICAL_YAW_THRUSTERS,
)
from sim.runtime.physics_runtime_setup import create_runtime_physics_setup


def create_runner_runtime_physics(
    *,
    args,
    mujoco_module,
    np_module,
    sim_profile,
    perf_cfg,
    initial_setup,
    control_setup,
    model_io,
    use_custom_hydrodynamics: bool,
    plant_replay_direct_rcout: bool,
    env_float,
    env_flag,
    to_float_array,
    log,
):
    return create_runtime_physics_setup(
        args=args,
        mujoco_module=mujoco_module,
        np_module=np_module,
        model=initial_setup.model,
        data=initial_setup.data,
        sim_profile=sim_profile,
        perf_cfg=perf_cfg,
        scene_fluid_density=initial_setup.scene_fluid_density,
        base_id=initial_setup.base_id,
        world_qpos_adr=initial_setup.world_qpos_adr,
        world_qvel_adr=initial_setup.world_qvel_adr,
        fluidcoef_dynamic_setup=initial_setup.fluidcoef_dynamic_setup,
        fluid_geom_names=initial_setup.fluid_geom_names,
        water_surface_z=initial_setup.water_surface_z,
        base_origin_world=initial_setup.base_origin_world,
        real_start_required=initial_setup.real_start_required,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        actuator_ids=model_io.actuator_ids,
        ctrlrange=model_io.ctrlrange,
        thruster_params_path=model_io.thruster_params_path,
        all_thruster_names=model_io.all_thruster_names,
        horizontal_order=model_io.horizontal_order,
        vertical_names=model_io.vertical_names,
        vertical_thrusters=PHYSICAL_VERTICAL_THRUSTERS,
        horizontal_thrusters=PHYSICAL_YAW_THRUSTERS,
        servo_map=list(ARDUSUB_VECTORED_6DOF_SERVO_MAP),
        servo_signs=list(ARDUSUB_VECTORED_6DOF_SERVO_SIGNS),
        plant_replay_direct_rcout=plant_replay_direct_rcout,
        ros_bridge_runtime=control_setup.ros_bridge_runtime,
        initial_depth_hold=initial_setup.initial_depth_hold,
        command_state=control_setup.command_state,
        thruster_air_force_scale=initial_setup.thruster_air_force_scale,
        thruster_immersion_half_height_m=initial_setup.thruster_immersion_half_height_m,
        env_float=env_float,
        env_flag=env_flag,
        to_float_array=to_float_array,
        log=log,
    )


__all__ = ["create_runner_runtime_physics"]
