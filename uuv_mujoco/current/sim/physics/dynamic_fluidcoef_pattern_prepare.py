"""Prepare one dynamic-fluidcoef pattern row before mutating arrays."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from sim.physics.dynamic_fluidcoef_pattern_matching import matching_fluid_geom_ids
from sim.physics.dynamic_fluidcoef_pattern_weights import (
    dynamic_fluidcoef_pattern_axis_weights,
    dynamic_fluidcoef_row_weights,
)
from sim.physics.dynamic_fluidcoef_setup_config import DynamicFluidcoefProfileConfig


@dataclass(frozen=True)
class DynamicFluidcoefPatternSetup:
    matching_geom_ids: list[int]
    ratio: np.ndarray
    row_weights: np.ndarray
    row_axis_weights: np.ndarray
    row_angular_axis_weights: np.ndarray


def dynamic_fluidcoef_reference_ratio(
    *,
    profile_cfg: DynamicFluidcoefProfileConfig,
    pattern: str,
    raw_reference_scale: object,
    fluidcoef_static_geom_scales: dict[str, np.ndarray],
    to_float_array: Callable[[object], np.ndarray | None],
) -> np.ndarray | None:
    reference_scale = to_float_array(raw_reference_scale)
    if reference_scale is None or reference_scale.size != 5:
        print(
            "[physics] ignoring dynamic_fluidcoef reference "
            f"for {pattern!r}: expected 5 values "
            "(blunt, slender, angular, Kutta, Magnus)",
            flush=True,
        )
        return None
    reference_scale = np.clip(reference_scale.astype(np.float64, copy=False), 0.0, 100.0)
    current_scale = fluidcoef_static_geom_scales.get(str(pattern))
    if current_scale is None:
        current_scale = np.ones(5, dtype=np.float64)
    ratio = np.divide(
        reference_scale,
        np.maximum(current_scale.astype(np.float64, copy=False), 1.0e-12),
    )
    return np.clip(ratio, profile_cfg.min_multiplier, profile_cfg.max_multiplier)


def prepare_dynamic_fluidcoef_pattern(
    *,
    profile_cfg: DynamicFluidcoefProfileConfig,
    pattern: str,
    raw_reference_scale: object,
    fluidcoef_static_geom_scales: dict[str, np.ndarray],
    fluid_geom_names: dict[int, str],
    to_float_array: Callable[[object], np.ndarray | None],
    to_float_matrix: Callable[[object, tuple[int, int]], np.ndarray | None],
) -> DynamicFluidcoefPatternSetup | None:
    ratio = dynamic_fluidcoef_reference_ratio(
        profile_cfg=profile_cfg,
        pattern=pattern,
        raw_reference_scale=raw_reference_scale,
        fluidcoef_static_geom_scales=fluidcoef_static_geom_scales,
        to_float_array=to_float_array,
    )
    if ratio is None:
        return None

    matching_geom_ids = matching_fluid_geom_ids(fluid_geom_names, pattern)
    if not matching_geom_ids:
        print(
            "[physics] warning: dynamic_fluidcoef pattern "
            f"{pattern!r} matched no fluid geoms",
            flush=True,
        )
        return None

    row_axis_weights, row_angular_axis_weights = dynamic_fluidcoef_pattern_axis_weights(
        profile_cfg,
        pattern,
        to_float_matrix,
    )
    return DynamicFluidcoefPatternSetup(
        matching_geom_ids=matching_geom_ids,
        ratio=ratio,
        row_weights=dynamic_fluidcoef_row_weights(profile_cfg, pattern, to_float_array),
        row_axis_weights=row_axis_weights,
        row_angular_axis_weights=row_angular_axis_weights,
    )


__all__ = [
    "DynamicFluidcoefPatternSetup",
    "dynamic_fluidcoef_reference_ratio",
    "prepare_dynamic_fluidcoef_pattern",
]
