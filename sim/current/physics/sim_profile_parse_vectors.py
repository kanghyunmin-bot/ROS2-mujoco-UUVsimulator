"""Vector parsers for simulation profile fields."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .sim_profile_parse_array import to_float_array


def vector_from_keys(
    payload: Mapping[str, Any],
    direct_key: str,
    split_prefix: str,
    default_linear: np.ndarray,
    default_angular: np.ndarray | None = None,
) -> np.ndarray:
    direct = to_float_array(payload.get(direct_key), 6)
    if direct is not None:
        return direct

    linear = to_float_array(payload.get(f"{split_prefix}_linear"), 3)
    if linear is None:
        linear = np.array(default_linear, dtype=np.float64)

    if default_angular is None:
        return linear

    angular = to_float_array(payload.get(f"{split_prefix}_angular"), 3)
    if angular is None:
        angular = np.array(default_angular, dtype=np.float64)

    return np.concatenate((linear, angular)).astype(np.float64, copy=False)


def vector3_from_keys(payload: Mapping[str, Any], *keys: str, default: tuple[float, float, float]) -> np.ndarray:
    for key in keys:
        values = to_float_array(payload.get(key), 3)
        if values is not None:
            return values
    return np.array(default, dtype=np.float64)


__all__ = ["vector3_from_keys", "vector_from_keys"]
