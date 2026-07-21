"""Thruster debug CSV contract helpers."""

from __future__ import annotations

from collections.abc import Iterable


def build_thruster_debug_header(thruster_names: Iterable[str]) -> list[str]:
    """Return the CSV header used by the MuJoCo thruster debug stream."""
    header = [
        "wall_mono_s",
        "sim_time",
        "lin_vel_body_x",
        "lin_vel_body_y",
        "lin_vel_body_z",
        "ang_vel_body_x",
        "ang_vel_body_y",
        "ang_vel_body_z",
        "base_depth_m",
        "base_vz_down_mps",
        "buoy_force_world_z",
        "weight_force_world_z",
        "net_static_force_world_z",
        "initial_depth_hold_active",
        "thr_force_body_x",
        "thr_force_body_y",
        "thr_force_body_z",
        "thr_torque_body_x",
        "thr_torque_body_y",
        "thr_torque_body_z",
        "thr_force_world_x",
        "thr_force_world_y",
        "thr_force_world_z",
        "thr_torque_world_x",
        "thr_torque_world_y",
        "thr_torque_world_z",
        "xfrc_force_world_x",
        "xfrc_force_world_y",
        "xfrc_force_world_z",
        "xfrc_torque_world_x",
        "xfrc_torque_world_y",
        "xfrc_torque_world_z",
    ]
    for prefix in (
        "qvel",
        "qfrc_applied",
        "qfrc_actuator",
        "qfrc_fluid",
        "qfrc_passive",
        "qfrc_bias",
        "qacc",
    ):
        header.extend(f"{prefix}_{idx}" for idx in range(6))
    header.extend(f"sitl_ch{idx}_pwm" for idx in range(1, 9))
    for thr_name in thruster_names:
        header.extend(
            [
                f"{thr_name}_servo_norm",
                f"{thr_name}_target",
                f"{thr_name}_state",
                f"{thr_name}_force",
                f"{thr_name}_direct_gain",
            ]
        )
    return header
