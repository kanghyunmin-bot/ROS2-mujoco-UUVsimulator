"""MuJoCo raycast helpers for Ping360 beam returns."""

from __future__ import annotations

import math

import mujoco
import numpy as np


def raycast(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    origin: np.ndarray,
    direction: np.ndarray,
    geomgroup: np.ndarray,
    base_body_id: int,
    cutoff: float,
) -> tuple[float | None, int]:
    geom_id = np.array([-1], dtype=np.int32)
    distance = _mj_ray_distance(model, data, origin, direction, geomgroup, base_body_id, geom_id)
    if distance < 0.0 or distance > cutoff or not math.isfinite(distance):
        return None, -1
    return distance, int(geom_id[0])


def _mj_ray_distance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    origin: np.ndarray,
    direction: np.ndarray,
    geomgroup: np.ndarray,
    base_body_id: int,
    geom_id: np.ndarray,
) -> float:
    try:
        return float(mujoco.mj_ray(model, data, origin, direction, geomgroup, 1, int(base_body_id), geom_id))
    except TypeError:
        return float(mujoco.mj_ray(model, data, origin, direction, None, 1, int(base_body_id), geom_id))


__all__ = ["raycast"]
