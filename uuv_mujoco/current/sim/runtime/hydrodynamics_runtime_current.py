"""MuJoCo ambient-current setup for hydrodynamics runtime."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np


def configure_mujoco_current(
    model,
    *,
    water_current_world,
    use_custom_hydrodynamics: bool,
    log: Callable[[str], None],
) -> None:
    if use_custom_hydrodynamics:
        return
    # MuJoCo's built-in ellipsoid fluid model uses opt.wind as the surrounding
    # flow velocity that is subtracted from body velocity.
    model.opt.wind[:] = water_current_world
    if np.linalg.norm(water_current_world) > 1.0e-9:
        log(
            "[physics] MuJoCo fluid ambient current via opt.wind: "
            f"{np.array2string(water_current_world, precision=3)} m/s"
        )


__all__ = ["configure_mujoco_current"]
