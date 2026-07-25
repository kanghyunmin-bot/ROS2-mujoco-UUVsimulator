"""Public attitude methods for MuJoCoBaseState."""

from __future__ import annotations

from sim.runtime.base_state_free_joint import set_free_joint_attitude_quat
from sim.runtime.pose_math import quat_wxyz_from_rpy_rad


class BaseStateAttitudeMixin:
    def set_base_attitude_rpy(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        reset_velocity: bool = True,
    ) -> None:
        """Set the free body attitude from ROS/ENU roll, pitch, yaw."""
        set_free_joint_attitude_quat(
            mujoco=self.mujoco,
            model=self.model,
            data=self.data,
            world_qpos_adr=self.world_qpos_adr,
            world_qvel_adr=self.world_qvel_adr,
            quat_wxyz=quat_wxyz_from_rpy_rad(roll, pitch, yaw),
            reset_velocity=reset_velocity,
        )


__all__ = ["BaseStateAttitudeMixin"]
