"""Ping360 beam-hit extraction from MuJoCo raycasts."""

from __future__ import annotations

from collections.abc import Iterator

import mujoco
import numpy as np

from .ping360_beam_model import angle_grad_to_rad, beam_directions_local, raycast
from .ping360_types import Ping360Config, Ping360EffectiveSettings


def iter_beam_returns(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    base_body_id: int,
    geomgroup: np.ndarray,
    config: Ping360Config,
    settings: Ping360EffectiveSettings,
    angle_grad: int,
) -> Iterator[tuple[float, int, float]]:
    if site_id < 0:
        return

    origin = np.asarray(data.site_xpos[site_id], dtype=np.float64).copy()
    site_rot = np.asarray(data.site_xmat[site_id], dtype=np.float64).reshape(3, 3).copy()
    for local_dir, weight in beam_directions_local(config, angle_grad_to_rad(angle_grad)):
        world_dir = site_rot @ local_dir
        norm = float(np.linalg.norm(world_dir))
        if norm <= 1.0e-12:
            continue
        world_dir = world_dir / norm
        dist, geom_id = raycast(
            model=model,
            data=data,
            origin=origin,
            direction=world_dir,
            geomgroup=geomgroup,
            base_body_id=base_body_id,
            cutoff=settings.effective_range_m,
        )
        if dist is None:
            continue
        if dist < config.min_range_m or dist > settings.effective_range_m:
            continue
        yield dist, geom_id, weight


__all__ = ["iter_beam_returns"]
