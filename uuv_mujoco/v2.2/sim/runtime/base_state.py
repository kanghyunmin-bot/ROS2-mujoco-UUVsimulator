"""MuJoCo base-body depth and Bar30 placement helpers."""

from __future__ import annotations

from dataclasses import dataclass

from .base_state_attitude_mixin import BaseStateAttitudeMixin
from .base_state_bar30_mixin import BaseStateBar30Mixin
from .base_state_free_joint_mixin import BaseStateFreeJointMixin
from .base_state_ids import lookup_base_state_ids


@dataclass
class MuJoCoBaseState(BaseStateFreeJointMixin, BaseStateBar30Mixin, BaseStateAttitudeMixin):
    """Mutable helpers for the vehicle free joint and Bar30 depth reference."""

    mujoco: object
    model: object
    data: object
    base_id: int
    world_joint_id: int
    world_qpos_adr: int
    world_qvel_adr: int
    water_surface_z: float
    bar30_site_id: int

    @classmethod
    def create(cls, *, mujoco, model, data, water_surface_z: float) -> "MuJoCoBaseState":
        """Lookup required MuJoCo ids and return a configured state helper."""
        ids = lookup_base_state_ids(mujoco=mujoco, model=model)
        return cls(
            mujoco=mujoco,
            model=model,
            data=data,
            base_id=ids.base_id,
            world_joint_id=ids.world_joint_id,
            world_qpos_adr=ids.world_qpos_adr,
            world_qvel_adr=ids.world_qvel_adr,
            water_surface_z=float(water_surface_z),
            bar30_site_id=ids.bar30_site_id,
        )


__all__ = ["MuJoCoBaseState"]
