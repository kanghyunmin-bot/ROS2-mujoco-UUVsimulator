"""Final callback wiring and result packaging for runtime physics setup."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from sim.runtime.physics_runtime_types import RuntimePhysicsSetup
from sim.runtime.physics_step_callbacks import build_step_physics_callbacks


def build_runtime_physics_setup_result(
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
    hydrostatic_context: Any,
    hydro_runtime: Any,
    thruster_actuator_runtime: Any,
    thruster_debug_runtime: Any,
    underwater_wrench_runtime: Any,
    sitl_servo_runtime: Any,
    thruster_param_runtime: Any,
    body_velocity_local: Callable[[], np.ndarray],
    ros_bridge_runtime: Any,
    env_flag: Callable[[str, bool], bool],
    log: Callable[[str], None],
) -> RuntimePhysicsSetup:
    callbacks = build_step_physics_callbacks(
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
        horizontal_allocator=hydrostatic_context.horizontal_allocator,
        hydro_runtime=hydro_runtime,
        thruster_actuator_runtime=thruster_actuator_runtime,
        thruster_debug_runtime=thruster_debug_runtime,
        underwater_wrench_runtime=underwater_wrench_runtime,
        sitl_servo_runtime=sitl_servo_runtime,
        thruster_param_runtime=thruster_param_runtime,
        body_velocity_local=body_velocity_local,
        vehicle_mass=hydrostatic_context.vehicle_mass,
        gravity=hydrostatic_context.gravity,
        ros_bridge_runtime=ros_bridge_runtime,
        env_flag=env_flag,
        log=log,
    )

    return RuntimePhysicsSetup(
        sitl_servo_runtime=sitl_servo_runtime,
        sitl_servo_pwm_values=sitl_servo_runtime.pwm_values,
        sitl_servo_timeout_s=sitl_servo_runtime.timeout_s,
        thruster_actuator_runtime=thruster_actuator_runtime,
        thruster_debug_runtime=thruster_debug_runtime,
        underwater_wrench_runtime=underwater_wrench_runtime,
        horizontal_allocator=hydrostatic_context.horizontal_allocator,
        thruster_site_ids=thruster_actuator_runtime.site_ids,
        thr_state=thruster_actuator_runtime.state,
        thr_target=thruster_actuator_runtime.target,
        thruster_force_max=hydro_runtime.thruster_force_max,
        vehicle_mass=hydrostatic_context.vehicle_mass,
        gravity=hydrostatic_context.gravity,
        thruster_update_due=callbacks.thruster_update_due,
        update_thruster_forces=callbacks.update_thruster_forces,
        update_propeller_visuals=callbacks.update_propeller_visuals,
        apply_direct_command_targets=callbacks.apply_direct_command_targets,
        apply_underwater_wrench=callbacks.apply_underwater_wrench,
        emit_thruster_debug=callbacks.emit_thruster_debug,
        enforce_descent_contract=callbacks.enforce_descent_contract,
    )


__all__ = ["build_runtime_physics_setup_result"]
