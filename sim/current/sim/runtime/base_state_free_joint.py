"""Free-joint qpos/qvel mutation helpers for the vehicle base body."""

from __future__ import annotations

import numpy as np


def base_origin_world(*, data, base_id: int) -> np.ndarray:
    return data.xpos[base_id].copy()


def reset_free_joint_velocity(*, data, world_qvel_adr: int) -> None:
    data.qvel[world_qvel_adr : world_qvel_adr + 6] = 0.0
    data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0


def set_free_joint_depth(
    *,
    mujoco,
    model,
    data,
    world_qpos_adr: int,
    world_qvel_adr: int,
    water_surface_z: float,
    depth_m: float,
    reset_velocity: bool,
) -> None:
    data.qpos[world_qpos_adr + 2] = float(water_surface_z) - float(depth_m)
    if reset_velocity:
        reset_free_joint_velocity(data=data, world_qvel_adr=world_qvel_adr)
    mujoco.mj_forward(model, data)


def set_free_joint_xy(
    *,
    mujoco,
    model,
    data,
    world_qpos_adr: int,
    world_qvel_adr: int,
    x_m: float,
    y_m: float,
    reset_velocity: bool,
) -> None:
    data.qpos[world_qpos_adr + 0] = float(x_m)
    data.qpos[world_qpos_adr + 1] = float(y_m)
    if reset_velocity:
        reset_free_joint_velocity(data=data, world_qvel_adr=world_qvel_adr)
    mujoco.mj_forward(model, data)


def set_free_joint_attitude_quat(
    *,
    mujoco,
    model,
    data,
    world_qpos_adr: int,
    world_qvel_adr: int,
    quat_wxyz: np.ndarray,
    reset_velocity: bool,
) -> None:
    data.qpos[world_qpos_adr + 3 : world_qpos_adr + 7] = quat_wxyz
    if reset_velocity:
        data.qvel[world_qvel_adr + 3 : world_qvel_adr + 6] = 0.0
        data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
    mujoco.mj_forward(model, data)


__all__ = [
    "base_origin_world",
    "reset_free_joint_velocity",
    "set_free_joint_attitude_quat",
    "set_free_joint_depth",
    "set_free_joint_xy",
]
