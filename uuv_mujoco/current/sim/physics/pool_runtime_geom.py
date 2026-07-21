"""MuJoCo pool-geometry lookup helpers."""

from __future__ import annotations


def pool_geom_id(model, mujoco_module, name: str) -> int:
    return int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_GEOM, name))


__all__ = ["pool_geom_id"]
