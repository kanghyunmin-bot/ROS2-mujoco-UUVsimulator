"""Evaluation helpers for runtime descent diagnostics."""

from __future__ import annotations

from typing import Sequence

from sim.runtime.descent_contract_logic import (
    classify_descent_cause,
    descent_vertical_pwm_delta,
    format_descent_contract_message,
)


def descent_guard_should_check(
    *,
    enabled: bool,
    initial_depth_hold_active: bool,
    sim_time: float,
    start_s: float,
    base_vz_down_mps: float,
    vz_down_mps: float,
) -> bool:
    if not enabled:
        return False
    if initial_depth_hold_active or float(sim_time) < float(start_s):
        return False
    return float(base_vz_down_mps) >= float(vz_down_mps)


def build_descent_contract_message(
    *,
    base_depth_m: float,
    base_vz_down_mps: float,
    vertical_pwm: Sequence[int],
    thruster_force_world_z: float,
    buoy_force_world_z: float,
    vehicle_mass: float,
    gravity: float,
) -> str:
    weight_force_world_z = -float(vehicle_mass * gravity)
    net_static_force_world_z = float(buoy_force_world_z + weight_force_world_z)
    cause = classify_descent_cause(
        vertical_pwm_delta=descent_vertical_pwm_delta(vertical_pwm),
        thruster_force_world_z=thruster_force_world_z,
        net_static_force_world_z=net_static_force_world_z,
    )
    return format_descent_contract_message(
        cause=cause,
        base_depth_m=base_depth_m,
        base_vz_down_mps=base_vz_down_mps,
        vertical_pwm=vertical_pwm,
        thruster_force_world_z=thruster_force_world_z,
        buoy_force_world_z=buoy_force_world_z,
        weight_force_world_z=weight_force_world_z,
        net_static_force_world_z=net_static_force_world_z,
    )


__all__ = ["build_descent_contract_message", "descent_guard_should_check"]
