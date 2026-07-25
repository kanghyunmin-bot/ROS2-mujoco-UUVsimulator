"""Initial Bar30 pressure calibration for real-start matching."""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

from sim.runtime.real_start import calibrate_bar30_surface_pressure_to_real_start


def calibrate_initial_bar30_surface_pressure(
    *,
    model: Any,
    base_state: Any,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
) -> None:
    calibrate_bar30_surface_pressure_to_real_start(
        env=os.environ,
        env_flag=env_flag,
        env_float=env_float,
        model_density=float(model.opt.density),
        bar30_depth_m=base_state.bar30_depth_now_m(),
        base_depth_m=float(base_state.water_surface_z - float(base_state.base_origin_world()[2])),
    )


__all__ = ["calibrate_initial_bar30_surface_pressure"]
