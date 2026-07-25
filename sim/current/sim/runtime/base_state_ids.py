"""MuJoCo id lookup for the vehicle base-state helper."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BaseStateIds:
    base_id: int
    world_joint_id: int
    world_qpos_adr: int
    world_qvel_adr: int
    bar30_site_id: int


def lookup_base_state_ids(*, mujoco, model) -> BaseStateIds:
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_id < 0:
        raise SystemExit("[runtime] base_link body not found in model")
    world_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint")
    if world_joint_id < 0:
        raise SystemExit("[runtime] world_joint free joint not found in model")
    bar30_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "bar30_site")
    return BaseStateIds(
        base_id=int(base_id),
        world_joint_id=int(world_joint_id),
        world_qpos_adr=int(model.jnt_qposadr[world_joint_id]),
        world_qvel_adr=int(model.jnt_dofadr[world_joint_id]),
        bar30_site_id=int(bar30_site_id),
    )


__all__ = ["BaseStateIds", "lookup_base_state_ids"]
