"""Initial hold pose/depth application helpers."""

from __future__ import annotations

from typing import Callable

import numpy as np

from .initial_hold_pose_apply import apply_captured_hold_pose, apply_hold_depth_fallback
from .initial_hold_pose_capture import capture_initial_hold_pose, coerce_body_vector


def apply_initial_hold_pose_or_depth(
    *,
    active: bool,
    depth_m: float | None,
    bar30_depth_m: float | None,
    pose_qpos: np.ndarray | None,
    data,
    mujoco,
    model,
    world_qpos_adr: int,
    world_qvel_adr: int,
    set_bar30_depth: Callable[..., None],
    set_base_depth: Callable[..., None],
) -> None:
    """Apply the initial depth hold pose/depth constraint if active."""
    if not active:
        return
    if depth_m is None and bar30_depth_m is None:
        return
    if apply_captured_hold_pose(
        pose_qpos,
        data=data,
        mujoco=mujoco,
        model=model,
        world_qpos_adr=world_qpos_adr,
        world_qvel_adr=world_qvel_adr,
    ):
        return
    apply_hold_depth_fallback(
        depth_m=depth_m,
        bar30_depth_m=bar30_depth_m,
        set_bar30_depth=set_bar30_depth,
        set_base_depth=set_base_depth,
    )


__all__ = ["apply_initial_hold_pose_or_depth", "capture_initial_hold_pose", "coerce_body_vector"]
