"""Runtime MuJoCo option scaling for scene fluid density and viscosity."""

from __future__ import annotations

from typing import Callable

import numpy as np

EnvFloat = Callable[[str, float], float]


def apply_fluid_option_scales(
    model,
    sim_profile: dict,
    *,
    env_float: EnvFloat,
) -> tuple[float, float]:
    """Apply density/viscosity runtime scales and return scene fluid values."""

    scene_fluid_density = float(model.opt.density)
    scene_fluid_viscosity = float(model.opt.viscosity)
    fluid_density_scale = float(
        np.clip(
            env_float("UUV_MJ_DENSITY_SCALE", float(sim_profile.get("mujoco_density_scale", 1.0))),
            0.0,
            10.0,
        )
    )
    fluid_viscosity_scale = float(
        np.clip(
            env_float("UUV_MJ_VISCOSITY_SCALE", float(sim_profile.get("mujoco_viscosity_scale", 1.0))),
            0.0,
            10.0,
        )
    )
    if abs(fluid_density_scale - 1.0) > 1.0e-12:
        scene_fluid_density *= fluid_density_scale
        print(
            "[physics] MuJoCo fluid density scale: "
            f"scale={fluid_density_scale:.6g}, density={scene_fluid_density:.6g}",
            flush=True,
        )
    if abs(fluid_viscosity_scale - 1.0) > 1.0e-12:
        scene_fluid_viscosity *= fluid_viscosity_scale
        print(
            "[physics] MuJoCo fluid viscosity scale: "
            f"scale={fluid_viscosity_scale:.6g}, viscosity={scene_fluid_viscosity:.6g}",
            flush=True,
        )
    return scene_fluid_density, scene_fluid_viscosity
