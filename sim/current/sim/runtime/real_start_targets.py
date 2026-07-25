"""Target extraction for real-start status payloads."""

from __future__ import annotations

import math

import numpy as np

from .real_start_types import EnvFloat, RealStartTargets


def load_real_start_targets(env_float: EnvFloat) -> RealStartTargets:
    target_base_depth = env_float("UUV_REAL_START_BASE_DEPTH_M", math.nan)
    target_depth = (
        target_base_depth
        if math.isfinite(target_base_depth)
        else env_float("UUV_REAL_START_DEPTH_M", math.nan)
    )
    depth_contract = "base_link" if math.isfinite(target_base_depth) else "bar30"
    return RealStartTargets(
        target_depth=float(target_depth),
        depth_contract=depth_contract,
        target_rpy=(
            env_float("UUV_REAL_START_ROLL_RAD", math.nan),
            env_float("UUV_REAL_START_PITCH_RAD", math.nan),
            env_float("UUV_REAL_START_YAW_RAD", math.nan),
        ),
        target_x=env_float("UUV_REAL_START_BASE_X_M", math.nan),
        target_y=env_float("UUV_REAL_START_BASE_Y_M", math.nan),
        target_pressure_pa=env_float("UUV_REAL_START_STATIC_PRESSURE_PA", math.nan),
        target_v=np.asarray(
            [
                env_float("UUV_REAL_START_BODY_VX_MPS", math.nan),
                env_float("UUV_REAL_START_BODY_VY_MPS", math.nan),
                env_float("UUV_REAL_START_BODY_VZ_MPS", math.nan),
            ],
            dtype=np.float64,
        ),
        target_w=np.asarray(
            [
                env_float("UUV_REAL_START_BODY_WX_RADPS", math.nan),
                env_float("UUV_REAL_START_BODY_WY_RADPS", math.nan),
                env_float("UUV_REAL_START_BODY_WZ_RADPS", math.nan),
            ],
            dtype=np.float64,
        ),
        source_t_s=env_float("UUV_REAL_START_SOURCE_T_S", math.nan),
        pressure_tol_pa=float(max(env_float("UUV_REAL_START_PRESSURE_TOL_PA", 25.0), 0.0)),
        xy_tol_m=float(max(env_float("UUV_REAL_START_XY_TOL_M", 0.02), 0.0)),
    )


__all__ = ["load_real_start_targets"]
