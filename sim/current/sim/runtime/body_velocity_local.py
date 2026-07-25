"""Body-frame velocity reader for MuJoCo runtime physics."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np


def body_velocity_local_factory(
    *,
    mujoco_module: Any,
    model: Any,
    data: Any,
    base_id: int,
) -> Callable[[], tuple[np.ndarray, np.ndarray]]:
    def body_velocity_local() -> tuple[np.ndarray, np.ndarray]:
        vel6 = np.zeros(6, dtype=np.float64)
        mujoco_module.mj_objectVelocity(
            model,
            data,
            mujoco_module.mjtObj.mjOBJ_BODY,
            int(base_id),
            vel6,
            1,
        )
        return vel6[3:].copy(), vel6[:3].copy()

    return body_velocity_local


__all__ = ["body_velocity_local_factory"]
