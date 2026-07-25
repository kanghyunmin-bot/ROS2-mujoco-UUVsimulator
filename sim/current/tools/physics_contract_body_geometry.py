"""MuJoCo body/geom lookup helpers for physics contract audits."""

from __future__ import annotations

import numpy as np

from physics_contract_mujoco import mujoco


def body_subtree_mass(model: mujoco.MjModel, body_id: int) -> float:
    body_parent = np.asarray(model.body_parentid, dtype=np.int32)
    total = 0.0
    for bid in range(1, model.nbody):
        cur = bid
        while cur > 0:
            if cur == body_id:
                total += float(model.body_mass[bid])
                break
            cur = int(body_parent[cur])
    return float(total)


def geom_local_top_z(model: mujoco.MjModel, body_id: int, geom_id: int) -> float:
    geom_body = int(model.geom_bodyid[geom_id])
    if geom_body != body_id:
        raise RuntimeError(
            f"fluid geom {geom_id} is not attached to base body; "
            "physics_contract_audit only supports base_link proxy geoms"
        )
    return float(model.geom_pos[geom_id, 2] + model.geom_size[geom_id, 2])


__all__ = ["body_subtree_mass", "geom_local_top_z"]
