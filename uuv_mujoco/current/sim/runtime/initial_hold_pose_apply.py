"""Apply helpers for initial hold pose/depth state."""

from __future__ import annotations

from typing import Callable

import numpy as np


def apply_captured_hold_pose(
    pose_qpos: np.ndarray | None,
    *,
    data,
    mujoco,
    model,
    world_qpos_adr: int,
    world_qvel_adr: int,
) -> bool:
    if pose_qpos is None:
        return False
    hold_pose = np.asarray(pose_qpos, dtype=np.float64).copy()
    if hold_pose.shape != (7,) or not np.all(np.isfinite(hold_pose)):
        return False
    data.qpos[world_qpos_adr : world_qpos_adr + 7] = hold_pose
    data.qvel[world_qvel_adr : world_qvel_adr + 6] = 0.0
    data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
    mujoco.mj_forward(model, data)
    return True


def apply_hold_depth_fallback(
    *,
    depth_m: float | None,
    bar30_depth_m: float | None,
    set_bar30_depth: Callable[..., None],
    set_base_depth: Callable[..., None],
) -> None:
    if bar30_depth_m is not None:
        set_bar30_depth(float(bar30_depth_m), reset_velocity=True)
        return
    if depth_m is not None:
        set_base_depth(float(depth_m), reset_velocity=True)


__all__ = ["apply_captured_hold_pose", "apply_hold_depth_fallback"]
