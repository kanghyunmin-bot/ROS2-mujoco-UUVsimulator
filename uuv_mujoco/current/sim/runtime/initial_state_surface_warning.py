"""Surface-hysteresis warning helpers for initial depth setup."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any


def initial_surface_clearance_m(env_float: Callable[[str, float], float]) -> float:
    surface_depth_m = abs(env_float("SITL_SURFACE_DEPTH", -10.0)) / 100.0
    return surface_depth_m + 0.05


def warn_initial_bar30_surface_hysteresis(
    *,
    args: Any,
    bar30_depth_m: float,
    env_float: Callable[[str, float], float],
    message_tail: str,
) -> None:
    surface_clear_m = initial_surface_clearance_m(env_float)
    if not args.sitl or bar30_depth_m >= surface_clear_m:
        return
    print(
        "[runtime] warning: initial Bar30 depth "
        f"{bar30_depth_m:.3f}m is inside ArduSub SURFACE_DEPTH hysteresis "
        f"(<{surface_clear_m:.3f}m). {message_tail}",
        flush=True,
    )


__all__ = ["initial_surface_clearance_m", "warn_initial_bar30_surface_hysteresis"]
