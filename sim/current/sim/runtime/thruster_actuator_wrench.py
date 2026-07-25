"""Wrench accumulation helpers for MuJoCo thruster force updates."""

from __future__ import annotations

import numpy as np


def accumulate_thruster_wrench(
    runtime,
    *,
    name: str,
    aid: int,
    sid: int,
    force: float,
    base_rot: np.ndarray,
    com_body: np.ndarray,
    reaction_torque_gain: float,
) -> None:
    fdir = runtime.model.actuator_gear[aid, :3]
    fdir = fdir / (np.linalg.norm(fdir) + 1e-9)
    r_body = runtime.model.site_pos[sid].copy() - com_body if sid >= 0 else np.zeros(3, dtype=np.float64)
    force_body = fdir * force
    runtime.last_force_body += force_body
    runtime.last_torque_body += np.cross(r_body, force_body)
    world_dir = base_rot @ fdir
    runtime.last_reaction_torque_world += (
        -runtime.prop_spin_sign.get(name, 1.0) * world_dir * force * reaction_torque_gain
    )
    yaw_torque_scale = float(
        runtime.yaw_torque_thruster_scales.get(name, runtime.yaw_torque_scale)
    )
    if abs(yaw_torque_scale - 1.0) > 1.0e-9 and name in runtime.yaw_thrusters and sid >= 0:
        tau_body = np.cross(r_body, fdir * force)
        extra_tau_body = np.array(
            [0.0, 0.0, tau_body[2] * (yaw_torque_scale - 1.0)],
            dtype=np.float64,
        )
        runtime.last_reaction_torque_world += base_rot @ extra_tau_body


__all__ = ["accumulate_thruster_wrench"]
