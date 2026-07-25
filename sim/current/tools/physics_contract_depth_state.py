"""Base-depth state helpers for physics contract audits."""

from __future__ import annotations

from physics_contract_mujoco import mujoco


def set_base_depth(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    world_qpos_adr: int,
    world_qvel_adr: int,
    water_surface_z: float,
    depth_m: float,
) -> None:
    data.qpos[world_qpos_adr + 2] = float(water_surface_z - depth_m)
    data.qvel[world_qvel_adr : world_qvel_adr + 6] = 0.0
    data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
    mujoco.mj_forward(model, data)


__all__ = ["set_base_depth"]
