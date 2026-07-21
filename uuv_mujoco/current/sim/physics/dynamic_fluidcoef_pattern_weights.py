"""Per-pattern weight parsing for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef_setup_config import DynamicFluidcoefProfileConfig
from sim.physics.dynamic_fluidcoef_types import (
    DEFAULT_FLUIDCOEF_ANGULAR_AXIS_WEIGHTS,
    DEFAULT_FLUIDCOEF_AXIS_WEIGHTS,
)


def dynamic_fluidcoef_row_weights(
    profile_cfg: DynamicFluidcoefProfileConfig,
    pattern: str,
    to_float_array: Callable[[object], np.ndarray | None],
) -> np.ndarray:
    raw_weights = profile_cfg.coefficient_load_weights.get(str(pattern), profile_cfg.default_weights)
    row_weights = to_float_array(raw_weights)
    if row_weights is None or row_weights.size != 5:
        row_weights = profile_cfg.default_weights
    return np.clip(row_weights.astype(np.float64, copy=False), 0.0, 5.0)


def dynamic_fluidcoef_axis_weights(
    raw_weights: object,
    default_weights: np.ndarray,
    to_float_matrix: Callable[[object, tuple[int, int]], np.ndarray | None],
) -> np.ndarray:
    row_axis_weights = to_float_matrix(raw_weights, (5, 3))
    if row_axis_weights is None:
        row_axis_weights = default_weights
    return np.clip(row_axis_weights.astype(np.float64, copy=False), 0.0, 5.0)


def dynamic_fluidcoef_pattern_axis_weights(
    profile_cfg: DynamicFluidcoefProfileConfig,
    pattern: str,
    to_float_matrix: Callable[[object, tuple[int, int]], np.ndarray | None],
) -> tuple[np.ndarray, np.ndarray]:
    return (
        dynamic_fluidcoef_axis_weights(
            profile_cfg.coefficient_axis_weights.get(str(pattern), []),
            DEFAULT_FLUIDCOEF_AXIS_WEIGHTS,
            to_float_matrix,
        ),
        dynamic_fluidcoef_axis_weights(
            profile_cfg.coefficient_angular_axis_weights.get(str(pattern), []),
            DEFAULT_FLUIDCOEF_ANGULAR_AXIS_WEIGHTS,
            to_float_matrix,
        ),
    )


__all__ = [
    "dynamic_fluidcoef_axis_weights",
    "dynamic_fluidcoef_pattern_axis_weights",
    "dynamic_fluidcoef_row_weights",
]
