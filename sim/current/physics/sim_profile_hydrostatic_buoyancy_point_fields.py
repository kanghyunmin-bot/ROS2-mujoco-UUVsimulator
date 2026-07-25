"""Field extraction helpers for profile buoyancy points."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .sim_profile_parse_common import to_float_array


def buoyancy_point_pos(item: Mapping[str, Any]) -> np.ndarray | None:
    return to_float_array(item.get("pos"), 3)


def buoyancy_point_scalars(item: Mapping[str, Any], default_half_height: float) -> tuple[float, float] | None:
    try:
        share = float(item.get("share", 0.0))
        half_height = float(item.get("half_height", default_half_height))
    except (TypeError, ValueError):
        return None
    if not np.isfinite(share) or share < 0.0:
        return None
    if not np.isfinite(half_height) or half_height <= 0.0:
        return None
    return share, half_height


def buoyancy_point_name(idx: int, item: Mapping[str, Any]) -> str:
    return str(item.get("name", f"buoyancy_point_{idx}")).strip() or f"buoyancy_point_{idx}"


__all__ = ["buoyancy_point_name", "buoyancy_point_pos", "buoyancy_point_scalars"]
