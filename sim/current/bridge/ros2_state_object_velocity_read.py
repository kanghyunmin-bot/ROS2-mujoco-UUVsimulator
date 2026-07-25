"""MuJoCo object velocity read helper."""

from __future__ import annotations

import mujoco
import numpy as np

from .ros2_state_kinematics_arrays import finite_vector_copy


def object_world_linear_velocity_enu_or_fallback(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    obj_type: int,
    obj_id: int,
    fallback_vel_enu: np.ndarray,
) -> np.ndarray:
    if obj_id >= 0:
        try:
            vel6 = np.zeros(6, dtype=np.float64)
            mujoco.mj_objectVelocity(model, data, obj_type, obj_id, vel6, 0)
            lin_enu = finite_vector_copy(vel6[3:6])
            if lin_enu is not None:
                return lin_enu
        except Exception:
            pass
    return np.array(fallback_vel_enu, dtype=np.float64).copy()


__all__ = ["object_world_linear_velocity_enu_or_fallback"]
