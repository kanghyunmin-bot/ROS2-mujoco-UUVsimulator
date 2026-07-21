"""Top-level MuJoCo fluid geom and coefficient runtime-scale application."""

from __future__ import annotations

import os
from typing import Callable

import numpy as np

from sim.physics.fluid_geom_common import ArrayParser, fluid_geom_mask, fluid_geom_name
from sim.physics.fluid_geom_size_runtime import apply_extra_geom_size_scale, apply_profile_geom_size_scales
from sim.physics.fluidcoef_scale_runtime import (
    apply_extra_fluidcoef_scale,
    apply_global_fluidcoef_scale,
    apply_per_geom_fluidcoef_scales,
)


def apply_fluid_geom_runtime_scales(
    model,
    mujoco_module,
    sim_profile: dict,
    *,
    to_float_array: ArrayParser,
    env_get: Callable[[str, str], str | None] = os.getenv,
) -> tuple[np.ndarray, dict[int, str], dict[str, np.ndarray]]:
    """Apply static MuJoCo fluid geom/coef scales and return derived geom metadata."""

    fluidcoef_scale = to_float_array(sim_profile.get("mujoco_fluidcoef_scale"))
    fluid_mask = fluid_geom_mask(model)
    fluid_geom_ids = np.flatnonzero(fluid_mask)
    fluid_geom_names = {int(geom_id): fluid_geom_name(model, mujoco_module, int(geom_id)) for geom_id in fluid_geom_ids}
    fluidcoef_static_geom_scales: dict[str, np.ndarray] = {}

    apply_profile_geom_size_scales(
        model,
        sim_profile,
        fluid_geom_ids,
        fluid_geom_names,
        to_float_array=to_float_array,
    )
    apply_extra_geom_size_scale(
        model,
        fluid_geom_ids,
        raw_text=str(env_get("UUV_MJ_FLUID_GEOM_SIZE_EXTRA_SCALE", "") or "").strip(),
        to_float_array=to_float_array,
    )
    apply_global_fluidcoef_scale(model, fluid_mask, fluid_geom_ids, fluidcoef_scale)
    apply_per_geom_fluidcoef_scales(
        model,
        sim_profile,
        fluid_geom_ids,
        fluid_geom_names,
        fluidcoef_static_geom_scales,
        to_float_array=to_float_array,
    )
    apply_extra_fluidcoef_scale(
        model,
        fluid_mask,
        fluid_geom_ids,
        raw_text=str(env_get("UUV_MJ_FLUIDCOEF_EXTRA_SCALE", "") or "").strip(),
    )
    return fluid_geom_ids, fluid_geom_names, fluidcoef_static_geom_scales


__all__ = ["apply_fluid_geom_runtime_scales"]
