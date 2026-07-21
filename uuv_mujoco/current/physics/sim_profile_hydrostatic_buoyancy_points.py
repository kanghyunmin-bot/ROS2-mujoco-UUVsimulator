"""Buoyancy-point parser for simulation profiles."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .sim_profile_hydrostatic_buoyancy_point_fields import (
    buoyancy_point_name,
    buoyancy_point_pos,
    buoyancy_point_scalars,
)
from .sim_profile_types import BuoyancyPoint


def parse_buoyancy_points(
    sim_profile: Mapping[str, Any],
    default_half_height: float,
) -> tuple[BuoyancyPoint, ...]:
    payload = sim_profile.get("buoyancy_points")
    if not isinstance(payload, list):
        return ()

    points: list[BuoyancyPoint] = []
    for idx, item in enumerate(payload):
        point = _parse_buoyancy_point(idx, item, default_half_height)
        if point is not None:
            points.append(point)
    return tuple(points)


def _parse_buoyancy_point(
    idx: int,
    item: Any,
    default_half_height: float,
) -> BuoyancyPoint | None:
    if not isinstance(item, Mapping):
        return None
    pos = buoyancy_point_pos(item)
    if pos is None:
        return None
    point_scalars = buoyancy_point_scalars(item, default_half_height)
    if point_scalars is None:
        return None
    share, half_height = point_scalars
    return BuoyancyPoint(
        name=buoyancy_point_name(idx, item),
        pos=pos.astype(np.float64, copy=True),
        share=share,
        half_height=half_height,
    )


__all__ = ["parse_buoyancy_points"]
