"""MuJoCo world kinematics helpers for Ros2Bridge."""

from __future__ import annotations

import mujoco
import numpy as np

from .ros2_state_body_velocity_read import body_cvel_world_linear_velocity_enu_or_zero
from .ros2_state_object_velocity_read import object_world_linear_velocity_enu_or_fallback
from .ros2_state_site_read import site_world_pos_enu_or_fallback


def _site_world_pos_enu(self, data: mujoco.MjData, site_id: int, fallback_pos_enu: np.ndarray) -> np.ndarray:
    return site_world_pos_enu_or_fallback(
        data=data,
        site_id=site_id,
        fallback_pos_enu=fallback_pos_enu,
    )


def _body_cvel_world_linear_velocity_enu(
    self,
    data: mujoco.MjData,
    body_id: int,
    body_rot_enu: np.ndarray,
) -> np.ndarray:
    return body_cvel_world_linear_velocity_enu_or_zero(
        data=data,
        body_id=body_id,
        body_rot_enu=body_rot_enu,
    )


def _object_world_linear_velocity_enu(
    self,
    data: mujoco.MjData,
    obj_type: int,
    obj_id: int,
    fallback_vel_enu: np.ndarray,
) -> np.ndarray:
    return object_world_linear_velocity_enu_or_fallback(
        model=self.model,
        data=data,
        obj_type=obj_type,
        obj_id=obj_id,
        fallback_vel_enu=fallback_vel_enu,
    )


__all__ = [
    "_site_world_pos_enu",
    "_body_cvel_world_linear_velocity_enu",
    "_object_world_linear_velocity_enu",
]
