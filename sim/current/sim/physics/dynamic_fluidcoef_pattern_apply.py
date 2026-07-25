"""Array mutation for prepared dynamic MuJoCo fluidcoef pattern rows."""

from __future__ import annotations

import numpy as np

from sim.physics.dynamic_fluidcoef_pattern_prepare import DynamicFluidcoefPatternSetup


def apply_dynamic_fluidcoef_pattern_arrays(
    *,
    pattern_setup: DynamicFluidcoefPatternSetup,
    base: np.ndarray,
    reference: np.ndarray,
    weights: np.ndarray,
    axis_weights: np.ndarray,
    angular_axis_weights: np.ndarray,
    active_geom_ids: set[int],
) -> None:
    idx = np.array(pattern_setup.matching_geom_ids, dtype=np.int32)
    reference[idx, :] = base[idx, :] * pattern_setup.ratio.reshape(1, 5)
    weights[idx, :] = pattern_setup.row_weights.reshape(1, 5)
    axis_weights[idx, :, :] = pattern_setup.row_axis_weights.reshape(1, 5, 3)
    angular_axis_weights[idx, :, :] = pattern_setup.row_angular_axis_weights.reshape(1, 5, 3)
    active_geom_ids.update(pattern_setup.matching_geom_ids)


__all__ = ["apply_dynamic_fluidcoef_pattern_arrays"]
