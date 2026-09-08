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
    base_id: int,
    base_rot: np.ndarray,
    com_body: np.ndarray,
    reaction_torque_gain: float,
) -> None:
    gear_direction = runtime.model.actuator_gear[aid, :3]
    gear_direction = gear_direction / (np.linalg.norm(gear_direction) + 1e-9)
    if sid >= 0:
        site_rotation_world = np.asarray(
            runtime.data.site_xmat[sid],
            dtype=np.float64,
        ).reshape(3, 3)
        world_dir = site_rotation_world @ gear_direction
        r_world = (
            np.asarray(runtime.data.site_xpos[sid], dtype=np.float64)
            - np.asarray(runtime.data.xipos[base_id], dtype=np.float64)
        )
        r_body = base_rot.T @ r_world
        fdir = base_rot.T @ world_dir
    else:
        fdir = gear_direction
        world_dir = base_rot @ fdir
        r_body = np.zeros(3, dtype=np.float64)
    force_body = fdir * force
    runtime.last_force_body += force_body
    runtime.last_torque_body += np.cross(r_body, force_body)
    if not hasattr(runtime, "last_reaction_torque_body"):
        runtime.last_reaction_torque_body = np.zeros(3, dtype=np.float64)
    reaction_torque_body = (
        -runtime.prop_spin_sign.get(name, 1.0)
        * fdir
        * force
        * reaction_torque_gain
    )
    runtime.last_reaction_torque_body += reaction_torque_body
    runtime.last_reaction_torque_world += base_rot @ reaction_torque_body
    yaw_torque_scale = float(
        runtime.yaw_torque_thruster_scales.get(name, runtime.yaw_torque_scale)
    )
    if abs(yaw_torque_scale - 1.0) > 1.0e-9 and name in runtime.yaw_thrusters and sid >= 0:
        tau_body = np.cross(r_body, fdir * force)
        extra_tau_body = np.array(
            [0.0, 0.0, tau_body[2] * (yaw_torque_scale - 1.0)],
            dtype=np.float64,
        )
        runtime.last_reaction_torque_body += extra_tau_body
        runtime.last_reaction_torque_world += base_rot @ extra_tau_body


def reaction_torque_world_for_rotation(runtime, base_rot: np.ndarray) -> np.ndarray:
    """Rotate the cached body-fixed reaction torque at the physics cadence."""

    torque_body = getattr(runtime, "last_reaction_torque_body", None)
    if torque_body is None:
        return np.asarray(runtime.last_reaction_torque_world, dtype=np.float64)
    return np.asarray(base_rot, dtype=np.float64) @ np.asarray(
        torque_body,
        dtype=np.float64,
    )


__all__ = ["accumulate_thruster_wrench", "reaction_torque_world_for_rotation"]
