"""Descent-contract guard helpers for step-time physics callbacks."""

from __future__ import annotations

import os
from typing import Any, Callable

from sim.runtime.descent_contract_guard import DescentContractGuard


def build_descent_guard(
    *,
    args: Any,
    np_module: Any,
    env_flag: Callable[[str, bool], bool],
) -> DescentContractGuard:
    return DescentContractGuard(
        enabled=bool(args.sitl and env_flag("UUV_MJ_DESCENT_CONTRACT_GUARD", True)),
        fail_fast=env_flag("UUV_MJ_DESCENT_CONTRACT_FAIL_FAST", False),
        vz_down_mps=float(
            np_module.clip(float(os.getenv("UUV_MJ_DESCENT_CONTRACT_VZ_DOWN_MPS", "0.05")), 0.005, 1.0)
        ),
        start_s=float(np_module.clip(float(os.getenv("UUV_MJ_DESCENT_CONTRACT_START_S", "2.0")), 0.0, 60.0)),
    )


def enforce_descent_contract(
    *,
    descent_guard: DescentContractGuard,
    data: Any,
    base_id: int,
    world_qvel_adr: int,
    water_surface_z: float,
    base_origin_world: Callable[[], Any],
    initial_depth_hold: dict[str, Any],
    sitl_servo_runtime: Any,
    thruster_actuator_runtime: Any,
    underwater_wrench_runtime: Any,
    vehicle_mass: float,
    gravity: float,
    log: Callable[[str], None],
) -> None:
    base_vz_down_mps = -float(data.qvel[world_qvel_adr + 2])
    vertical_pwm = [int(value) for value in sitl_servo_runtime.pwm_values[4:8]]
    base_rot = data.xmat[base_id].reshape(3, 3)
    thruster_force_world_z = float((base_rot @ thruster_actuator_runtime.last_force_body)[2])
    message = descent_guard.check(
        initial_depth_hold_active=bool(initial_depth_hold["active"]),
        sim_time=float(data.time),
        base_depth_m=water_surface_z - float(base_origin_world()[2]),
        base_vz_down_mps=base_vz_down_mps,
        vertical_pwm=vertical_pwm,
        thruster_force_world_z=thruster_force_world_z,
        buoy_force_world_z=float(underwater_wrench_runtime.last_buoy_force[2]),
        vehicle_mass=vehicle_mass,
        gravity=gravity,
    )
    if message:
        log(message)


__all__ = ["build_descent_guard", "enforce_descent_contract"]
