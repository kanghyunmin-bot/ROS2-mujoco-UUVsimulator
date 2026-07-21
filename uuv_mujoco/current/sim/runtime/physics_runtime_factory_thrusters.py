"""Thruster runtime factory steps."""

from __future__ import annotations

from sim.runtime.physics_runtime_factory_context import PhysicsRuntimeFactoryContext
from sim.runtime.physics_runtime_thrusters import (
    create_thruster_actuator_runtime,
    create_thruster_debug_runtime,
    create_thruster_param_and_servo_runtimes,
)


def create_runtime_thruster_bundle(ctx: PhysicsRuntimeFactoryContext):
    return create_thruster_param_and_servo_runtimes(
        args=ctx.args,
        all_thruster_names=ctx.all_thruster_names,
        servo_map=ctx.servo_map,
        servo_signs=ctx.servo_signs,
        plant_replay_direct_rcout=ctx.plant_replay_direct_rcout,
        ros_bridge_runtime=ctx.ros_bridge_runtime,
        thruster_params_path=ctx.thruster_params_path,
        sim_profile=ctx.sim_profile,
        vertical_thrusters=ctx.vertical_thrusters,
        horizontal_thrusters=ctx.horizontal_thrusters,
        perf_cfg=ctx.perf_cfg,
        log=ctx.log,
    )


def create_runtime_thruster_actuator(ctx: PhysicsRuntimeFactoryContext, thruster_param_runtime, hydro_runtime):
    return create_thruster_actuator_runtime(
        model=ctx.model,
        data=ctx.data,
        mujoco_module=ctx.mujoco_module,
        actuator_ids=ctx.actuator_ids,
        ctrlrange=ctx.ctrlrange,
        all_thruster_names=ctx.all_thruster_names,
        thruster_global=thruster_param_runtime.global_params,
        thruster_scale=thruster_param_runtime.scale,
        thruster_direct_scale=thruster_param_runtime.direct_scale,
        thruster_reverse_asymmetry=thruster_param_runtime.reverse_asymmetry,
        thruster_tau_up=thruster_param_runtime.tau_up,
        thruster_tau_down=thruster_param_runtime.tau_down,
        perf_cfg=ctx.perf_cfg,
        thruster_force_max=hydro_runtime.thruster_force_max,
        water_surface_z=ctx.water_surface_z,
        thruster_air_force_scale=ctx.thruster_air_force_scale,
        thruster_immersion_half_height_m=ctx.thruster_immersion_half_height_m,
        buoyancy_model=hydro_runtime.buoyancy_model,
        yaw_torque_scale=hydro_runtime.yaw_torque_scale,
        yaw_torque_thruster_scales=hydro_runtime.yaw_torque_thruster_scales,
        yaw_thrusters=ctx.horizontal_thrusters,
        spin_gain=hydro_runtime.spin_gain,
    )


def create_runtime_thruster_debug(ctx: PhysicsRuntimeFactoryContext):
    return create_thruster_debug_runtime(all_thruster_names=ctx.all_thruster_names, log=ctx.log)


__all__ = [
    "create_runtime_thruster_bundle",
    "create_runtime_thruster_actuator",
    "create_runtime_thruster_debug",
]
