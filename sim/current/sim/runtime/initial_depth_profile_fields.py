"""Profile field helpers for automatic initial-depth candidates."""

from __future__ import annotations

from typing import Iterable

import numpy as np

from .initial_depth_profile import vec3_from_profile


def profile_item_name(item: dict, fallback: str) -> str:
    return str(item.get("name", fallback))


def profile_item_pos(item: dict, keys: Iterable[str]) -> np.ndarray | None:
    for key in keys:
        pos = vec3_from_profile(item.get(key))
        if pos is not None:
            return pos
    return None


def profile_item_size(item: dict) -> np.ndarray | None:
    return vec3_from_profile(item.get("size"))


def nonnegative_half_height(value: object, default: float = 0.0) -> float:
    try:
        half_z = float(value)
    except (TypeError, ValueError):
        half_z = float(default)
    if not np.isfinite(half_z) or half_z < 0.0:
        return 0.0
    return half_z


__all__ = [
    "nonnegative_half_height",
    "profile_item_name",
    "profile_item_pos",
    "profile_item_size",
]
