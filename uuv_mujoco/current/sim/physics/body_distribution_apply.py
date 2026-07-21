"""Apply distributed body component values to a MuJoCo model."""

from __future__ import annotations

import numpy as np


def apply_composite_body_to_model(
    *,
    mujoco,
    model,
    data,
    base_id: int,
    world_qpos_adr: int,
    world_qvel_adr: int,
    total_mass: float,
    composite_com: np.ndarray,
    composite_inertia: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray]:
    old_mass = float(model.body_mass[base_id])
    old_com = model.body_ipos[base_id].copy()
    old_inertia = model.body_inertia[base_id].copy()
    saved_qpos = data.qpos[world_qpos_adr : world_qpos_adr + 7].copy()
    saved_qvel = data.qvel[world_qvel_adr : world_qvel_adr + 6].copy()

    model.body_mass[base_id] = total_mass
    model.body_ipos[base_id, :] = composite_com
    model.body_inertia[base_id, :] = np.maximum(composite_inertia, 1e-6)
    if hasattr(mujoco, "mj_setConst"):
        mujoco.mj_setConst(model, data)
    data.qpos[world_qpos_adr : world_qpos_adr + 7] = saved_qpos
    data.qvel[world_qvel_adr : world_qvel_adr + 6] = saved_qvel
    mujoco.mj_forward(model, data)
    return old_mass, old_com, old_inertia


__all__ = ["apply_composite_body_to_model"]
