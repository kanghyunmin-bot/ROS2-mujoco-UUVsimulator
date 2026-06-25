"""MuJoCo site-position read helper."""

from __future__ import annotations

import mujoco
import numpy as np

from .ros2_state_kinematics_arrays import finite_vector_copy


def site_world_pos_enu_or_fallback(
    *,
    data: mujoco.MjData,
    site_id: int,
    fallback_pos_enu: np.ndarray,
) -> np.ndarray:
    if site_id < 0:
        return fallback_pos_enu
    try:
        pos_enu = finite_vector_copy(data.site_xpos[site_id])
        if pos_enu is not None:
            return pos_enu
    except Exception:
        pass
    return fallback_pos_enu


__all__ = ["site_world_pos_enu_or_fallback"]
