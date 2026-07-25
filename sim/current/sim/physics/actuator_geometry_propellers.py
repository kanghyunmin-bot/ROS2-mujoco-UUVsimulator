"""Propeller joint mapping helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence


@dataclass(frozen=True)
class PropellerJointMaps:
    qpos_adr: dict[str, int]
    dof_adr: dict[str, int]
    spin_sign: dict[str, float]


def propeller_joint_maps(
    *,
    model: Any,
    mujoco_module: Any,
    names: Sequence[str],
) -> PropellerJointMaps:
    qpos_adr: dict[str, int] = {}
    dof_adr: dict[str, int] = {}
    spin_sign: dict[str, float] = {}
    for name in names:
        joint_id = int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_JOINT, f"prop_{name}_j"))
        if joint_id >= 0:
            qpos_adr[name] = int(model.jnt_qposadr[joint_id])
            dof_adr[name] = int(model.jnt_dofadr[joint_id])
            spin_sign[name] = 1.0 if name.endswith(("lf", "rr")) else -1.0
    return PropellerJointMaps(qpos_adr=qpos_adr, dof_adr=dof_adr, spin_sign=spin_sign)


__all__ = ["PropellerJointMaps", "propeller_joint_maps"]
