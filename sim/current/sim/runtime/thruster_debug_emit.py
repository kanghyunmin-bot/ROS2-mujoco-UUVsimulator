"""Single-row writer for MuJoCo thruster debug CSV output."""

from __future__ import annotations

from typing import TextIO

import numpy as np

from sim.runtime.thruster_debug_rows import (
    BaseOriginReader,
    BodyVelocityReader,
    build_thruster_debug_values,
    format_thruster_debug_values,
)


def emit_thruster_debug_row(
    file: TextIO,
    *,
    thruster_names: list[str],
    mujoco_module: object,
    model: object,
    data: object,
    base_id: int,
    world_qvel_adr: int,
    water_surface_z: float,
    vehicle_mass: float,
    gravity: float,
    base_origin_world: BaseOriginReader,
    body_velocity_local: BodyVelocityReader,
    last_buoy_force: np.ndarray,
    initial_depth_hold_active: bool,
    last_thruster_force_body: np.ndarray,
    last_thruster_torque_body: np.ndarray,
    sitl_servo_pwm_values: list[int],
    sitl_servo_cmd_norm: dict[str, float],
    thr_target: dict[str, float],
    thr_state: dict[str, float],
    thruster_force_cmd: dict[str, float],
    thruster_direct_scale: dict[str, float],
) -> None:
    """Write the current force breakdown using the established sample contract."""
    # Keep the force breakdown contract identical to the historical writer:
    # sample after ctrl/xfrc are staged and before integration.
    mujoco_module.mj_forward(model, data)

    values = build_thruster_debug_values(
        thruster_names=thruster_names,
        data=data,
        base_id=base_id,
        world_qvel_adr=world_qvel_adr,
        water_surface_z=water_surface_z,
        vehicle_mass=vehicle_mass,
        gravity=gravity,
        base_origin_world=base_origin_world,
        body_velocity_local=body_velocity_local,
        last_buoy_force=last_buoy_force,
        initial_depth_hold_active=initial_depth_hold_active,
        last_thruster_force_body=last_thruster_force_body,
        last_thruster_torque_body=last_thruster_torque_body,
        sitl_servo_pwm_values=sitl_servo_pwm_values,
        sitl_servo_cmd_norm=sitl_servo_cmd_norm,
        thr_target=thr_target,
        thr_state=thr_state,
        thruster_force_cmd=thruster_force_cmd,
        thruster_direct_scale=thruster_direct_scale,
    )
    file.write(format_thruster_debug_values(values))


__all__ = ["emit_thruster_debug_row"]
