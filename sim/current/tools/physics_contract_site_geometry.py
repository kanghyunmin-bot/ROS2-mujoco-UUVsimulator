"""MuJoCo site/fluid-geom helpers for physics contract audits."""

from __future__ import annotations

from physics_contract_mujoco import mujoco


def site_local_z(model: mujoco.MjModel, name: str) -> float | None:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if sid < 0:
        return None
    return float(model.site_pos[sid, 2])


def fluid_geom_ids(model: mujoco.MjModel) -> list[int]:
    ids: list[int] = []
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith("fluid_"):
            ids.append(gid)
    return ids


__all__ = ["fluid_geom_ids", "site_local_z"]
