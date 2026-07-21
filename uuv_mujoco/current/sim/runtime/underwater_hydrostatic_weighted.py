"""Weighted buoyancy point/component hydrostatic wrench helpers."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_hydrostatic_accumulator import (
    accumulate_weighted_hydrostatic,
    weighted_hydrostatic_result,
)
from sim.runtime.underwater_hydrostatic_samples import body_component_samples, buoyancy_point_samples
from sim.runtime.underwater_wrench_types import HydrostaticWrenchResult


def buoyancy_points_wrench(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    base_origin: np.ndarray,
    com: np.ndarray,
    cob: np.ndarray,
) -> HydrostaticWrenchResult:
    hs = runtime.hydrostatic
    samples = getattr(runtime, "cached_buoyancy_point_samples", None)
    if samples is None:
        samples = buoyancy_point_samples(hs.active_buoyancy_points, hs)
    return accumulate_weighted_hydrostatic(
        runtime,
        samples,
        base_rot=base_rot,
        base_origin=base_origin,
        com=com,
        cob=cob,
    )


def body_components_wrench(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    base_origin: np.ndarray,
    com: np.ndarray,
    cob: np.ndarray,
) -> HydrostaticWrenchResult:
    hs = runtime.hydrostatic
    samples = getattr(runtime, "cached_body_component_samples", None)
    if samples is None:
        samples = body_component_samples(hs.active_body_components, hs)
    return accumulate_weighted_hydrostatic(
        runtime,
        samples,
        base_rot=base_rot,
        base_origin=base_origin,
        com=com,
        cob=cob,
    )


__all__ = ["body_components_wrench", "buoyancy_points_wrench", "weighted_hydrostatic_result"]
