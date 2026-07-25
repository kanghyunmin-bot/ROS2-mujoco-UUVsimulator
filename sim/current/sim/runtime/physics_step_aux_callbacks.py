"""Auxiliary wrench, debug, and guard callbacks for one MuJoCo physics step."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from sim.runtime.course_buoy_runtime import CourseBuoyRuntime
from sim.runtime.env import env_float
from sim.runtime.physics_step_debug import emit_thruster_debug as emit_thruster_debug_payload
from sim.runtime.physics_step_descent import (
    build_descent_guard,
    enforce_descent_contract as enforce_descent_contract_payload,
)


@dataclass
class AuxStepCallbacks:
    apply_underwater_wrench: Callable[[float], None]
    emit_thruster_debug: Callable[[], None]
    enforce_descent_contract: Callable[[], None]


def build_aux_step_callbacks(
    *,
    args: Any,
    mujoco_module: Any,
    np_module: Any,
    model: Any,
    data: Any,
    base_id: int,
    world_qvel_adr: int,
    water_surface_z: float,
    base_origin_world: Callable[[], np.ndarray],
    initial_depth_hold: dict[str, Any],
    underwater_wrench_runtime: Any,
    thruster_debug_runtime: Any,
    sitl_servo_runtime: Any,
    thruster_actuator_runtime: Any,
    thruster_param_runtime: Any,
    body_velocity_local: Callable[[], tuple[np.ndarray, np.ndarray]],
    vehicle_mass: float,
    gravity: float,
    ros_bridge_runtime: Any,
    env_flag: Callable[[str, bool], bool],
    log: Callable[[str], None],
) -> AuxStepCallbacks:
    descent_guard = build_descent_guard(args=args, np_module=np_module, env_flag=env_flag)
    course_buoy_runtime = CourseBuoyRuntime.from_model(
        mujoco_module=mujoco_module,
        model=model,
        data=data,
        water_surface_z=water_surface_z,
        water_current_world=underwater_wrench_runtime.hydrodynamics.water_current_world,
        env_float=env_float,
        env_flag=env_flag,
        log=log,
    )
    active_bridge = ros_bridge_runtime.get() if ros_bridge_runtime is not None else None
    if active_bridge is not None:
        setattr(active_bridge, "_course_buoy_runtime", course_buoy_runtime)

    def apply_underwater_wrench(dt: float) -> None:
        underwater_wrench_runtime.apply(dt)
        course_buoy_runtime.apply(dt)

    def emit_thruster_debug() -> None:
        emit_thruster_debug_payload(
            thruster_debug_runtime=thruster_debug_runtime,
            mujoco_module=mujoco_module,
            model=model,
            data=data,
            base_id=base_id,
            world_qvel_adr=world_qvel_adr,
            water_surface_z=water_surface_z,
            vehicle_mass=vehicle_mass,
            gravity=gravity,
            base_origin_world=base_origin_world,
            body_velocity_local=body_velocity_local,
            underwater_wrench_runtime=underwater_wrench_runtime,
            initial_depth_hold=initial_depth_hold,
            thruster_actuator_runtime=thruster_actuator_runtime,
            sitl_servo_runtime=sitl_servo_runtime,
            thruster_param_runtime=thruster_param_runtime,
        )

    def enforce_descent_contract() -> None:
        enforce_descent_contract_payload(
            descent_guard=descent_guard,
            data=data,
            base_id=base_id,
            world_qvel_adr=world_qvel_adr,
            water_surface_z=water_surface_z,
            base_origin_world=base_origin_world,
            initial_depth_hold=initial_depth_hold,
            sitl_servo_runtime=sitl_servo_runtime,
            thruster_actuator_runtime=thruster_actuator_runtime,
            underwater_wrench_runtime=underwater_wrench_runtime,
            vehicle_mass=vehicle_mass,
            gravity=gravity,
            log=log,
        )

    return AuxStepCallbacks(
        apply_underwater_wrench=apply_underwater_wrench,
        emit_thruster_debug=emit_thruster_debug,
        enforce_descent_contract=enforce_descent_contract,
    )


__all__ = ["AuxStepCallbacks", "build_aux_step_callbacks"]
