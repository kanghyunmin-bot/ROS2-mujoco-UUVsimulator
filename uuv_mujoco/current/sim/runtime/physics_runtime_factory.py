"""Factory that wires runtime physics components together."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from sim.runtime.physics_runtime_finalize import build_runtime_physics_setup_result
from sim.runtime.physics_runtime_factory_context import PhysicsRuntimeFactoryContext
from sim.runtime.physics_runtime_factory_hydro import (
    apply_runtime_thruster_geometry,
    create_hydrodynamics_runtime,
    create_runtime_hydrostatic_context,
    create_underwater_runtime,
)
from sim.runtime.physics_runtime_factory_thrusters import (
    create_runtime_thruster_actuator,
    create_runtime_thruster_bundle,
    create_runtime_thruster_debug,
)
from sim.runtime.physics_runtime_types import RuntimePhysicsSetup


def create_runtime_physics_setup(
    *,
    args: Any,
    mujoco_module: Any,
    np_module: Any,
    model: Any,
    data: Any,
    sim_profile: dict[str, Any],
    perf_cfg: dict[str, Any],
    scene_fluid_density: float,
    base_id: int,
    world_qpos_adr: int,
    world_qvel_adr: int,
    fluidcoef_dynamic_setup: Any,
    fluid_geom_names: dict[int, str],
    water_surface_z: float,
    base_origin_world: Callable[[], np.ndarray],
    real_start_required: bool,
    use_custom_hydrodynamics: bool,
    actuator_ids: dict[str, int],
    ctrlrange: dict[str, tuple[float, float]],
    thruster_params_path: Any,
    all_thruster_names: list[str],
    horizontal_order: list[str],
    vertical_names: list[str],
    vertical_thrusters: list[str],
    horizontal_thrusters: list[str],
    servo_map: list[int],
    servo_signs: list[float],
    plant_replay_direct_rcout: bool,
    ros_bridge_runtime: Any,
    initial_depth_hold: dict[str, Any],
    command_state: Any,
    thruster_air_force_scale: float,
    thruster_immersion_half_height_m: float,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
    to_float_array: Callable[[str, np.ndarray], np.ndarray],
    log: Callable[[str], None],
) -> RuntimePhysicsSetup:
    ctx = PhysicsRuntimeFactoryContext(**locals())
    hydrostatic_context = create_runtime_hydrostatic_context(ctx)
    apply_runtime_thruster_geometry(ctx)
    thruster_bundle = create_runtime_thruster_bundle(ctx)
    thruster_param_runtime = thruster_bundle.thruster_param_runtime
    sitl_servo_runtime = thruster_bundle.sitl_servo_runtime

    hydro_runtime = create_hydrodynamics_runtime(ctx, hydrostatic_context)
    thruster_actuator_runtime = create_runtime_thruster_actuator(ctx, thruster_param_runtime, hydro_runtime)
    underwater_wrench_runtime, body_velocity_local = create_underwater_runtime(
        ctx,
        hydrostatic_context,
        hydro_runtime,
        thruster_actuator_runtime,
    )
    thruster_debug_runtime = create_runtime_thruster_debug(ctx)
    return build_runtime_physics_setup_result(
        args=args,
        mujoco_module=mujoco_module,
        np_module=np_module,
        model=model,
        data=data,
        base_id=base_id,
        world_qvel_adr=world_qvel_adr,
        water_surface_z=water_surface_z,
        base_origin_world=base_origin_world,
        initial_depth_hold=initial_depth_hold,
        command_state=command_state,
        horizontal_order=horizontal_order,
        vertical_names=vertical_names,
        hydrostatic_context=hydrostatic_context,
        hydro_runtime=hydro_runtime,
        thruster_actuator_runtime=thruster_actuator_runtime,
        thruster_debug_runtime=thruster_debug_runtime,
        underwater_wrench_runtime=underwater_wrench_runtime,
        sitl_servo_runtime=sitl_servo_runtime,
        thruster_param_runtime=thruster_param_runtime,
        body_velocity_local=body_velocity_local,
        ros_bridge_runtime=ros_bridge_runtime,
        env_flag=env_flag,
        log=log,
    )


__all__ = ["create_runtime_physics_setup"]
