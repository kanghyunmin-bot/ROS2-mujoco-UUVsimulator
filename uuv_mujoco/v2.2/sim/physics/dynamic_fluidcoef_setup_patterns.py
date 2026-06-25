"""Pattern loop helpers for dynamic MuJoCo fluid coefficient setup."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef_setup_config import DynamicFluidcoefProfileConfig
from sim.physics.dynamic_fluidcoef_setup_rows import apply_dynamic_fluidcoef_pattern


def apply_dynamic_fluidcoef_patterns(
    *,
    profile_cfg: DynamicFluidcoefProfileConfig,
    base: np.ndarray,
    reference: np.ndarray,
    weights: np.ndarray,
    axis_weights: np.ndarray,
    angular_axis_weights: np.ndarray,
    fluidcoef_static_geom_scales: dict[str, np.ndarray],
    fluid_geom_names: dict[int, str],
    to_float_array: Callable[[object], np.ndarray | None],
    to_float_matrix: Callable[[object, tuple[int, int]], np.ndarray | None],
) -> set[int]:
    active_geom_ids: set[int] = set()
    for pattern, raw_reference_scale in profile_cfg.reference_geom_scales.items():
        apply_dynamic_fluidcoef_pattern(
            profile_cfg=profile_cfg,
            pattern=str(pattern),
            raw_reference_scale=raw_reference_scale,
            base=base,
            reference=reference,
            weights=weights,
            axis_weights=axis_weights,
            angular_axis_weights=angular_axis_weights,
            fluidcoef_static_geom_scales=fluidcoef_static_geom_scales,
            fluid_geom_names=fluid_geom_names,
            active_geom_ids=active_geom_ids,
            to_float_array=to_float_array,
            to_float_matrix=to_float_matrix,
        )
    return active_geom_ids


__all__ = ["apply_dynamic_fluidcoef_patterns"]
