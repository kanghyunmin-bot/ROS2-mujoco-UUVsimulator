"""Per-pattern setup orchestration for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef_pattern_apply import apply_dynamic_fluidcoef_pattern_arrays
from sim.physics.dynamic_fluidcoef_pattern_logging import log_dynamic_fluidcoef_pattern_loaded
from sim.physics.dynamic_fluidcoef_pattern_matching import matching_fluid_geom_ids
from sim.physics.dynamic_fluidcoef_pattern_prepare import prepare_dynamic_fluidcoef_pattern
from sim.physics.dynamic_fluidcoef_setup_config import DynamicFluidcoefProfileConfig


def apply_dynamic_fluidcoef_pattern(
    *,
    profile_cfg: DynamicFluidcoefProfileConfig,
    pattern: str,
    raw_reference_scale: object,
    base: np.ndarray,
    reference: np.ndarray,
    weights: np.ndarray,
    axis_weights: np.ndarray,
    angular_axis_weights: np.ndarray,
    fluidcoef_static_geom_scales: dict[str, np.ndarray],
    fluid_geom_names: dict[int, str],
    active_geom_ids: set[int],
    to_float_array: Callable[[object], np.ndarray | None],
    to_float_matrix: Callable[[object, tuple[int, int]], np.ndarray | None],
) -> None:
    pattern_setup = prepare_dynamic_fluidcoef_pattern(
        profile_cfg=profile_cfg,
        pattern=pattern,
        raw_reference_scale=raw_reference_scale,
        fluidcoef_static_geom_scales=fluidcoef_static_geom_scales,
        fluid_geom_names=fluid_geom_names,
        to_float_array=to_float_array,
        to_float_matrix=to_float_matrix,
    )
    if pattern_setup is None:
        return

    apply_dynamic_fluidcoef_pattern_arrays(
        pattern_setup=pattern_setup,
        base=base,
        reference=reference,
        weights=weights,
        axis_weights=axis_weights,
        angular_axis_weights=angular_axis_weights,
        active_geom_ids=active_geom_ids,
    )
    log_dynamic_fluidcoef_pattern_loaded(
        pattern=pattern,
        pattern_setup=pattern_setup,
        fluid_geom_names=fluid_geom_names,
    )


__all__ = ["apply_dynamic_fluidcoef_pattern", "matching_fluid_geom_ids"]
