"""Setup builder for MuJoCo dynamic fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef_setup_arrays import (
    allocate_dynamic_fluidcoef_arrays,
    build_dynamic_fluidcoef_setup_result,
)
from sim.physics.dynamic_fluidcoef_setup_config import load_dynamic_fluidcoef_profile_config
from sim.physics.dynamic_fluidcoef_setup_enable import dynamic_fluidcoef_enabled_after_patterns
from sim.physics.dynamic_fluidcoef_setup_patterns import apply_dynamic_fluidcoef_patterns
from sim.physics.dynamic_fluidcoef_types import DynamicFluidcoefSetup


def build_dynamic_fluidcoef_setup(
    *,
    model,
    sim_profile: dict,
    fluid_model: str,
    fluid_geom_ids: np.ndarray,
    fluid_geom_names: dict[int, str],
    fluidcoef_static_geom_scales: dict[str, np.ndarray],
    env_flag: Callable[[str, bool], bool],
    to_float_array: Callable[[object], np.ndarray | None],
    to_float_matrix: Callable[[object, tuple[int, int]], np.ndarray | None],
) -> DynamicFluidcoefSetup:
    """Build the dynamic MuJoCo fluid coefficient state without touching loops."""

    profile_cfg = load_dynamic_fluidcoef_profile_config(
        sim_profile=sim_profile,
        fluid_model=fluid_model,
        fluid_geom_ids=fluid_geom_ids,
        env_flag=env_flag,
        to_float_array=to_float_array,
    )
    base, current, reference, weights, axis_weights, angular_axis_weights = allocate_dynamic_fluidcoef_arrays(model)
    active_geom_ids: set[int] = set()
    if not profile_cfg.enabled:
        return build_dynamic_fluidcoef_setup_result(
            cfg=profile_cfg.cfg,
            enabled=profile_cfg.enabled,
            base=base,
            current=current,
            reference=reference,
            weights=weights,
            axis_weights=axis_weights,
            angular_axis_weights=angular_axis_weights,
            active_geom_ids=active_geom_ids,
        )

    active_geom_ids = apply_dynamic_fluidcoef_patterns(
        profile_cfg=profile_cfg,
        base=base,
        reference=reference,
        weights=weights,
        axis_weights=axis_weights,
        angular_axis_weights=angular_axis_weights,
        fluidcoef_static_geom_scales=fluidcoef_static_geom_scales,
        fluid_geom_names=fluid_geom_names,
        to_float_array=to_float_array,
        to_float_matrix=to_float_matrix,
    )
    enabled = dynamic_fluidcoef_enabled_after_patterns(
        profile_cfg=profile_cfg,
        active_geom_ids=active_geom_ids,
    )

    return build_dynamic_fluidcoef_setup_result(
        cfg=profile_cfg.cfg,
        enabled=enabled,
        base=base,
        current=current,
        reference=reference,
        weights=weights,
        axis_weights=axis_weights,
        angular_axis_weights=angular_axis_weights,
        active_geom_ids=active_geom_ids,
    )
