"""Sample conversion helpers for weighted hydrostatic buoyancy points."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class WeightedHydrostaticSample:
    volume_pos: np.ndarray
    force_pos: np.ndarray
    half_height: float
    share: float


def buoyancy_point_samples(points: list[Any], hs: Any) -> tuple[WeightedHydrostaticSample, ...]:
    total_share = float(sum(point.share for point in points))
    if total_share <= 1e-9:
        total_share = float(len(points))
    return tuple(
        WeightedHydrostaticSample(
            volume_pos=point.pos.copy(),
            force_pos=_force_position(point.pos, hs),
            half_height=float(point.half_height),
            share=float(point.share) / max(total_share, 1e-9),
        )
        for point in points
    )


def body_component_samples(components: list[Any], hs: Any) -> tuple[WeightedHydrostaticSample, ...]:
    total_share = float(sum(component.buoyancy_share for component in components))
    if total_share <= 1e-9:
        total_share = float(sum(component.mass for component in components))
    return tuple(
        WeightedHydrostaticSample(
            volume_pos=component.buoyancy_pos.copy(),
            force_pos=_force_position(component.buoyancy_pos, hs),
            half_height=float(max(component.size[2], 1e-4)),
            share=float(component.buoyancy_share) / max(total_share, 1e-9),
        )
        for component in components
    )


def _force_position(volume_pos: np.ndarray, hs: Any) -> np.ndarray:
    force_pos = volume_pos.copy()
    force_pos[0] += hs.cob_longitudinal_offset
    force_pos[2] += hs.cob_vertical_offset
    return force_pos


__all__ = [
    "WeightedHydrostaticSample",
    "body_component_samples",
    "buoyancy_point_samples",
]
