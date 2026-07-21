"""Primitive MuJoCo user-scene geometry helpers."""

from __future__ import annotations

import numpy as np


def scene_has_capacity(user_scn) -> bool:
    return bool(user_scn.ngeom < user_scn.maxgeom)


def add_arrow_geom(user_scn, mujoco, start, direction, magnitude, rgba, thickness: float = 0.02) -> None:
    length = 0.08 + 0.01 * abs(float(magnitude))
    end = start + direction * length
    add_arrow_segment_geom(user_scn, mujoco, start, end, rgba, thickness)


def add_arrow_segment_geom(user_scn, mujoco, start, end, rgba, thickness: float = 0.02) -> None:
    if not scene_has_capacity(user_scn):
        return
    geom = user_scn.geoms[user_scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        np.zeros(3),
        np.zeros(3),
        np.eye(3).flatten(),
        np.array(rgba, dtype=np.float32),
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        thickness,
        start,
        end,
    )
    user_scn.ngeom += 1


def add_sphere_geom(user_scn, mujoco, position, radius: float, rgba) -> None:
    if not scene_has_capacity(user_scn):
        return
    geom = user_scn.geoms[user_scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, 0.0, 0.0]),
        position,
        np.eye(3).flatten(),
        np.array(rgba, dtype=np.float32),
    )
    user_scn.ngeom += 1


def add_label_geom(user_scn, mujoco, text: str, position, rgba) -> None:
    if not scene_has_capacity(user_scn):
        return
    geom = user_scn.geoms[user_scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_LABEL,
        np.zeros(3),
        position,
        np.eye(3).flatten(),
        np.array(rgba, dtype=np.float32),
    )
    geom.size[:] = np.array([0.03, 0.03, 0.03])
    geom.label = text
    user_scn.ngeom += 1


__all__ = [
    "add_arrow_geom",
    "add_arrow_segment_geom",
    "add_label_geom",
    "add_sphere_geom",
    "scene_has_capacity",
]
