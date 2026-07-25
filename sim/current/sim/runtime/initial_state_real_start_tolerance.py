"""Real-start tolerance parsing for initial-state contracts."""

from __future__ import annotations

from collections.abc import Callable


def real_start_tolerances(
    *,
    env_float: Callable[[str, float], float],
) -> tuple[float, float, float]:
    return (
        float(max(env_float("UUV_REAL_START_DEPTH_TOL_M", 0.02), 0.0)),
        float(max(env_float("UUV_REAL_START_ATTITUDE_TOL_RAD", 0.02), 0.0)),
        float(max(env_float("UUV_REAL_START_VELOCITY_TOL_MPS", 0.02), 0.0)),
    )


__all__ = ["real_start_tolerances"]
