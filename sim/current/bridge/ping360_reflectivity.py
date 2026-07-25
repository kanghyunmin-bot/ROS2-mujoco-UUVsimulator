"""Ping360 geom-name reflectivity defaults."""

from __future__ import annotations

import mujoco


def geom_reflectivity(model: mujoco.MjModel, geom_id: int) -> float:
    if geom_id < 0:
        return 0.0
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
    if "wall" in name:
        return 230.0
    if "floor" in name:
        return 190.0
    if "pool" in name:
        return 180.0
    return 150.0


__all__ = ["geom_reflectivity"]
