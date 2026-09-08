"""Row construction for MuJoCo thruster debug output."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.runtime.thruster_debug_channel_values import append_servo_pwm_values, append_thruster_command_values
from sim.runtime.thruster_debug_core_values import build_core_thruster_debug_values
from sim.runtime.thruster_debug_joint_values import free_joint_values


BodyVelocityReader = Callable[[], tuple[np.ndarray, np.ndarray]]
BaseOriginReader = Callable[[], np.ndarray]


def build_thruster_debug_values(
    *,
    thruster_names: list[str],
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
    thruster_diagnostics: dict | None = None,
) -> list[float]:
    values = build_core_thruster_debug_values(
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
    )
    append_servo_pwm_values(values, sitl_servo_pwm_values)
    append_thruster_command_values(
        values,
        thruster_names=thruster_names,
        sitl_servo_cmd_norm=sitl_servo_cmd_norm,
        thr_target=thr_target,
        thr_state=thr_state,
        thruster_force_cmd=thruster_force_cmd,
        thruster_direct_scale=thruster_direct_scale,
    )
    diagnostics = thruster_diagnostics or {}
    values.append(float(diagnostics.get("supply_voltage_v", float("nan"))))
    for name in thruster_names:
        for field in ("static_force_n", "immersion_scale", "inflow_multiplier", "effective_gain"):
            values.append(float(diagnostics.get(field, {}).get(name, float("nan"))))
    return values


def format_thruster_debug_values(values: list[float]) -> str:
    return ",".join(f"{value:.9g}" for value in values) + "\n"


__all__ = [
    "BaseOriginReader",
    "BodyVelocityReader",
    "build_thruster_debug_values",
    "format_thruster_debug_values",
    "free_joint_values",
]
