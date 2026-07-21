"""Public free-joint methods for MuJoCoBaseState."""

from __future__ import annotations

import numpy as np

from sim.runtime.base_state_free_joint import (
    base_origin_world as _base_origin_world,
    reset_free_joint_velocity as _reset_free_joint_velocity,
    set_free_joint_depth,
    set_free_joint_xy,
)


class BaseStateFreeJointMixin:
    def base_origin_world(self) -> np.ndarray:
        """Shared rigid-body reference used by SITL and buoyancy/depth helpers."""
        return _base_origin_world(data=self.data, base_id=self.base_id)

    def reset_free_joint_velocity(self) -> None:
        """Clear the world free-joint velocity and acceleration slices."""
        _reset_free_joint_velocity(data=self.data, world_qvel_adr=self.world_qvel_adr)

    def set_base_depth(self, depth_m: float, reset_velocity: bool = True) -> None:
        """Move the free body origin to a given positive-down depth."""
        set_free_joint_depth(
            mujoco=self.mujoco,
            model=self.model,
            data=self.data,
            world_qpos_adr=self.world_qpos_adr,
            world_qvel_adr=self.world_qvel_adr,
            water_surface_z=self.water_surface_z,
            depth_m=depth_m,
            reset_velocity=reset_velocity,
        )

    def set_base_position_xy(self, x_m: float, y_m: float, reset_velocity: bool = True) -> None:
        """Move the free body origin to a given horizontal ENU position."""
        set_free_joint_xy(
            mujoco=self.mujoco,
            model=self.model,
            data=self.data,
            world_qpos_adr=self.world_qpos_adr,
            world_qvel_adr=self.world_qvel_adr,
            x_m=x_m,
            y_m=y_m,
            reset_velocity=reset_velocity,
        )


__all__ = ["BaseStateFreeJointMixin"]
