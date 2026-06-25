"""Bar30 datum calibration for real-start runs."""

from __future__ import annotations

import math
import os

from .real_start_types import EnvFlag, EnvFloat


def calibrate_bar30_surface_pressure_to_real_start(
    *,
    env: os._Environ[str],
    env_flag: EnvFlag,
    env_float: EnvFloat,
    model_density: float,
    bar30_depth_m: float,
    base_depth_m: float,
) -> None:
    """Calibrate simulated Bar30 pressure datum to the real-start sample."""

    if not env_flag("UUV_REAL_START_STATE", False):
        return
    if env_flag("UUV_BAR30_SURFACE_PRESSURE_USER_SET", False):
        print(
            "[runtime] real-start Bar30 pressure datum: keeping user-supplied "
            "ROS2_UUV_BAR30_SURFACE_PRESSURE_PA",
            flush=True,
        )
        return
    if not env_flag("UUV_REAL_START_MATCH_BAR30_PRESSURE_TO_POSE", True):
        return
    target_pressure_pa = env_float("UUV_REAL_START_STATIC_PRESSURE_PA", math.nan)
    if not math.isfinite(target_pressure_pa):
        print(
            "[runtime] warning: real-start Bar30 pressure datum unavailable "
            "(missing UUV_REAL_START_STATIC_PRESSURE_PA)",
            flush=True,
        )
        return
    rho = env_float("ROS2_UUV_BAR30_WATER_DENSITY", float(model_density))
    gravity = env_float("ROS2_UUV_BAR30_GRAVITY", 9.80665)
    if not all(math.isfinite(v) for v in (rho, gravity, bar30_depth_m)):
        return
    surface_pressure_pa = float(target_pressure_pa - rho * gravity * max(0.0, bar30_depth_m))
    env["ROS2_UUV_BAR30_SURFACE_PRESSURE_PA"] = f"{surface_pressure_pa:.9f}"
    generated_pressure_pa = float(surface_pressure_pa + rho * gravity * max(0.0, bar30_depth_m))
    print(
        "[runtime] real-start Bar30 pressure datum calibrated: "
        f"surface={surface_pressure_pa:.3f}Pa "
        f"bar30_depth={bar30_depth_m:.3f}m "
        f"base_depth={base_depth_m:.3f}m "
        f"pressure={generated_pressure_pa:.3f}Pa "
        f"target={target_pressure_pa:.3f}Pa",
        flush=True,
    )


__all__ = ["calibrate_bar30_surface_pressure_to_real_start"]
