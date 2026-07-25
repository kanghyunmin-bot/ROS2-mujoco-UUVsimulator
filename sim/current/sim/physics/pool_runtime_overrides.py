"""Runtime pool geometry overrides for plant replay and smoke runs."""

from __future__ import annotations

from typing import Callable

import numpy as np

from sim.physics.pool_runtime_depth import apply_pool_depth_override
from sim.physics.pool_runtime_xy import apply_pool_xy_scale

EnvFloat = Callable[[str, float], float]


def apply_pool_runtime_overrides(
    model,
    mujoco_module,
    *,
    env_float: EnvFloat,
    run_mode: str,
) -> None:
    """Apply runtime pool depth and XY scale overrides."""

    pool_depth_override_m = float(env_float("UUV_POOL_DEPTH_M", 0.0))
    if pool_depth_override_m > 0.0:
        apply_pool_depth_override(model, mujoco_module, pool_depth_override_m)

    pool_xy_scale_default = 1.0
    pool_xy_scale = float(np.clip(env_float("UUV_POOL_XY_SCALE", pool_xy_scale_default), 1.0, 100.0))
    if pool_xy_scale <= 1.0 + 1.0e-9:
        return
    apply_pool_xy_scale(model, mujoco_module, pool_xy_scale)
