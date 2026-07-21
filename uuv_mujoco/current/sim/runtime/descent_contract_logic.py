"""Descent contract classification and report formatting."""

from __future__ import annotations

from collections.abc import Sequence


def descent_vertical_pwm_delta(vertical_pwm: Sequence[int]) -> int:
    return max((abs(int(v) - 1500) for v in vertical_pwm), default=0)


def classify_descent_cause(
    *,
    vertical_pwm_delta: int,
    thruster_force_world_z: float,
    net_static_force_world_z: float,
) -> str:
    if vertical_pwm_delta <= 12 and abs(thruster_force_world_z) <= 1.0:
        if net_static_force_world_z < -1.0:
            return "physics_negative_buoyancy_or_partial_submergence"
        return "neutral_pwm_with_existing_down_velocity"
    if thruster_force_world_z < -1.0:
        return "controller_or_mapping_is_commanding_down_force"
    return "controller_is_braking_or_force_sign_needs_review"


def format_descent_contract_message(
    *,
    cause: str,
    base_depth_m: float,
    base_vz_down_mps: float,
    vertical_pwm: Sequence[int],
    thruster_force_world_z: float,
    buoy_force_world_z: float,
    weight_force_world_z: float,
    net_static_force_world_z: float,
) -> str:
    return (
        "[descent-contract] "
        f"cause={cause} "
        f"depth={base_depth_m:.3f}m "
        f"vz_down={base_vz_down_mps:+.3f}m/s "
        f"json_pwm5_8={tuple(int(v) for v in vertical_pwm)} "
        f"thruster_force_z={thruster_force_world_z:+.3f}N "
        f"buoy_z={float(buoy_force_world_z):+.3f}N "
        f"weight_z={weight_force_world_z:+.3f}N "
        f"net_static_z={net_static_force_world_z:+.3f}N"
    )


__all__ = [
    "classify_descent_cause",
    "descent_vertical_pwm_delta",
    "format_descent_contract_message",
]
