"""MuJoCo body velocity read helper."""

from __future__ import annotations

import mujoco
import numpy as np

from .ros2_state_kinematics_arrays import finite_vector_copy, zero_velocity_enu


def body_cvel_world_linear_velocity_enu_or_zero(
    *,
    data: mujoco.MjData,
    body_id: int,
    body_rot_enu: np.ndarray,
) -> np.ndarray:
    del body_rot_enu
    if body_id >= 0:
        try:
            vel6 = np.zeros(6, dtype=np.float64)
            mujoco.mj_objectVelocity(
                data.model,
                data,
                mujoco.mjtObj.mjOBJ_BODY,
                int(body_id),
                vel6,
                0,
            )
            lin_enu = finite_vector_copy(vel6[3:6])
            if lin_enu is not None:
                return lin_enu
        except Exception:
            pass
    try:
        cvel = np.array(data.cvel[body_id], dtype=np.float64)
        if cvel.size >= 6:
            lin_enu = finite_vector_copy(cvel[3:6])
            if lin_enu is not None:
                return lin_enu
    except Exception:
        pass
    return zero_velocity_enu()


__all__ = ["body_cvel_world_linear_velocity_enu_or_zero"]
