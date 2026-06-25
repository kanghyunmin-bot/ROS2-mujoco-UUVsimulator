"""Step-time physics callbacks used by the MuJoCo runtime loop."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from sim.physics.horizontal_allocator import HorizontalAllocator
from sim.runtime.body_velocity_local import body_velocity_local_factory
from sim.runtime.physics_step_callback_types import StepPhysicsCallbacks
from sim.runtime.physics_step_aux_callbacks import build_aux_step_callbacks
from sim.runtime.physics_step_thruster_callbacks import build_thruster_step_callbacks
from sim.runtime.thruster_actuator_runtime import ThrusterActuatorRuntime
from sim.runtime.thruster_debug_runtime import ThrusterDebugRuntime
from sim.runtime.thruster_param_runtime import ThrusterParameterRuntime
from sim.runtime.underwater_wrench_runtime import UnderwaterWrenchRuntime


def build_step_physics_callbacks(
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
    command_state: Any,
    horizontal_order: list[str],
    vertical_names: list[str],
    horizontal_allocator: HorizontalAllocator,
    hydro_runtime: Any,
    thruster_actuator_runtime: ThrusterActuatorRuntime,
    thruster_debug_runtime: ThrusterDebugRuntime,
    underwater_wrench_runtime: UnderwaterWrenchRuntime,
    sitl_servo_runtime: Any,
    thruster_param_runtime: ThrusterParameterRuntime,
    body_velocity_local: Callable[[], tuple[np.ndarray, np.ndarray]],
    vehicle_mass: float,
    gravity: float,
    env_flag: Callable[[str, bool], bool],
    log: Callable[[str], None],
) -> StepPhysicsCallbacks:
    thruster_callbacks = build_thruster_step_callbacks(
        model=model,
        data=data,
        base_id=base_id,
        command_state=command_state,
        horizontal_order=horizontal_order,
        vertical_names=vertical_names,
        horizontal_allocator=horizontal_allocator,
        hydro_runtime=hydro_runtime,
        thruster_actuator_runtime=thruster_actuator_runtime,
    )
    aux_callbacks = build_aux_step_callbacks(
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
        underwater_wrench_runtime=underwater_wrench_runtime,
        thruster_debug_runtime=thruster_debug_runtime,
        sitl_servo_runtime=sitl_servo_runtime,
        thruster_actuator_runtime=thruster_actuator_runtime,
        thruster_param_runtime=thruster_param_runtime,
        body_velocity_local=body_velocity_local,
        vehicle_mass=vehicle_mass,
        gravity=gravity,
        env_flag=env_flag,
        log=log,
    )

    return StepPhysicsCallbacks(
        thruster_update_due=thruster_callbacks.thruster_update_due,
        update_thruster_forces=thruster_callbacks.update_thruster_forces,
        update_propeller_visuals=thruster_callbacks.update_propeller_visuals,
        apply_direct_command_targets=thruster_callbacks.apply_direct_command_targets,
        apply_underwater_wrench=aux_callbacks.apply_underwater_wrench,
        emit_thruster_debug=aux_callbacks.emit_thruster_debug,
        enforce_descent_contract=aux_callbacks.enforce_descent_contract,
    )
