"""Empirical hydrodynamic pitch, lift, and heave terms."""

from __future__ import annotations

from typing import Any

import numpy as np


def apply_empirical_pitch_lift_heave(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    rel_lin_vel_body: np.ndarray,
    buoyancy_submerged: float,
) -> None:
    hyd = runtime.hydrodynamics
    data = runtime.data
    base_id = int(runtime.base_id)

    if abs(hyd.hydro_pitch_moment_coeff) > 1.0e-9:
        forward_speed = float(rel_lin_vel_body[0])
        pitch_tau_body = np.array(
            [0.0, hyd.hydro_pitch_moment_coeff * forward_speed * abs(forward_speed), 0.0],
            dtype=np.float64,
        )
        data.xfrc_applied[base_id, 3:6] += base_rot @ pitch_tau_body
    if abs(hyd.hydro_vertical_lift_coeff) > 1.0e-9:
        forward_speed = float(rel_lin_vel_body[0])
        lift_speed = max(abs(forward_speed) - float(hyd.hydro_vertical_lift_deadband_mps), 0.0)
        lift_term = float(np.sign(forward_speed)) * lift_speed ** float(hyd.hydro_vertical_lift_power)
        lift_force_body = np.array(
            [0.0, 0.0, hyd.hydro_vertical_lift_coeff * lift_term],
            dtype=np.float64,
        )
        data.xfrc_applied[base_id, 0:3] += base_rot @ lift_force_body
    if (
        abs(hyd.hydro_yawrate_heave_pos_coeff) > 1.0e-9
        or abs(hyd.hydro_yawrate_heave_neg_coeff) > 1.0e-9
    ):
        forward_speed = float(rel_lin_vel_body[0])
        yaw_rate = float(runtime.data.qvel[int(runtime.world_qvel_adr) + 5])
        speed_term = max(abs(forward_speed) - float(hyd.hydro_yawrate_heave_speed_deadband_mps), 0.0)
        yaw_deadband = float(hyd.hydro_yawrate_heave_yaw_deadband_radps)
        yaw_heave_force_z = 0.0
        if yaw_rate > yaw_deadband:
            yaw_heave_force_z = (
                -hyd.hydro_yawrate_heave_pos_coeff
                * speed_term
                * (yaw_rate - yaw_deadband)
            )
        elif yaw_rate < -yaw_deadband:
            yaw_heave_force_z = (
                hyd.hydro_yawrate_heave_neg_coeff
                * speed_term
                * ((-yaw_rate) - yaw_deadband)
            )
        if abs(yaw_heave_force_z) > 1.0e-12:
            yaw_heave_force_body = np.array([0.0, 0.0, yaw_heave_force_z], dtype=np.float64)
            data.xfrc_applied[base_id, 0:3] += base_rot @ yaw_heave_force_body
    if hyd.heave_extra_damping_n_per_mps > 1.0e-9 and not hyd.cfd_dynamic_wrench_owns_z:
        heave_force_body = np.array(
            [
                0.0,
                0.0,
                -hyd.heave_extra_damping_n_per_mps
                * float(np.clip(buoyancy_submerged, 0.0, 1.0))
                * float(rel_lin_vel_body[2]),
            ],
            dtype=np.float64,
        )
        data.xfrc_applied[base_id, 0:3] += base_rot @ heave_force_body


__all__ = ["apply_empirical_pitch_lift_heave"]
