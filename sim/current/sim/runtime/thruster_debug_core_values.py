"""Core row values for MuJoCo thruster debug output."""

from __future__ import annotations

import time

import numpy as np

from sim.runtime.thruster_debug_joint_values import free_joint_values


def build_core_thruster_debug_values(
    *,
    data: object,
    base_id: int,
    world_qvel_adr: int,
    water_surface_z: float,
    vehicle_mass: float,
    gravity: float,
    base_origin_world,
    body_velocity_local,
    last_buoy_force: np.ndarray,
    initial_depth_hold_active: bool,
    last_thruster_force_body: np.ndarray,
    last_thruster_torque_body: np.ndarray,
) -> list[float]:
    lin_vel_body, ang_vel_body = body_velocity_local()
    base_depth_m = float(water_surface_z) - float(base_origin_world()[2])
    base_vz_down_mps = -float(data.qvel[int(world_qvel_adr) + 2])
    weight_force_world_z = -float(vehicle_mass * gravity)
    net_static_force_world_z = float(last_buoy_force[2] + weight_force_world_z)
    base_rot = data.xmat[int(base_id)].reshape(3, 3)
    thruster_force_world = base_rot @ last_thruster_force_body
    thruster_torque_world = base_rot @ last_thruster_torque_body
    values: list[float] = [
        time.monotonic(),
        float(data.time),
        *lin_vel_body.tolist(),
        *ang_vel_body.tolist(),
        base_depth_m,
        base_vz_down_mps,
        float(last_buoy_force[2]),
        weight_force_world_z,
        net_static_force_world_z,
        1.0 if initial_depth_hold_active else 0.0,
        *last_thruster_force_body.tolist(),
        *last_thruster_torque_body.tolist(),
        *thruster_force_world.tolist(),
        *thruster_torque_world.tolist(),
        *data.xfrc_applied[int(base_id), 0:6].astype(float, copy=False).tolist(),
    ]
    for attr_name in (
        "qvel",
        "qfrc_applied",
        "qfrc_actuator",
        "qfrc_fluid",
        "qfrc_passive",
        "qfrc_bias",
        "qacc",
    ):
        values.extend(free_joint_values(data, world_qvel_adr=int(world_qvel_adr), attr_name=attr_name).tolist())
    return values


__all__ = ["build_core_thruster_debug_values"]
