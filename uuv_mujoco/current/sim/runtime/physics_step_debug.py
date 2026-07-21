"""Thruster debug emission helper for step-time physics callbacks."""

from __future__ import annotations

from typing import Any, Callable


def emit_thruster_debug(
    *,
    thruster_debug_runtime: Any,
    mujoco_module: Any,
    model: Any,
    data: Any,
    base_id: int,
    world_qvel_adr: int,
    water_surface_z: float,
    vehicle_mass: float,
    gravity: float,
    base_origin_world: Callable[[], Any],
    body_velocity_local: Callable[[], Any],
    underwater_wrench_runtime: Any,
    initial_depth_hold: dict[str, Any],
    thruster_actuator_runtime: Any,
    sitl_servo_runtime: Any,
    thruster_param_runtime: Any,
) -> None:
    thruster_debug_runtime.emit(
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
        last_buoy_force=underwater_wrench_runtime.last_buoy_force,
        initial_depth_hold_active=bool(initial_depth_hold["active"]),
        last_thruster_force_body=thruster_actuator_runtime.last_force_body,
        last_thruster_torque_body=thruster_actuator_runtime.last_torque_body,
        sitl_servo_pwm_values=sitl_servo_runtime.pwm_values,
        sitl_servo_cmd_norm=sitl_servo_runtime.cmd_norm,
        thr_target=thruster_actuator_runtime.target,
        thr_state=thruster_actuator_runtime.state,
        thruster_force_cmd=thruster_actuator_runtime.force_cmd,
        thruster_direct_scale=thruster_param_runtime.direct_scale,
    )


__all__ = ["emit_thruster_debug"]
