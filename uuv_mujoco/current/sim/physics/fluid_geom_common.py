"""Common helpers for MuJoCo fluid geom runtime scaling."""

from __future__ import annotations

import fnmatch
from typing import Callable, Optional

import numpy as np

ArrayParser = Callable[[object], Optional[np.ndarray]]


def fluid_geom_name(model, mujoco_module, geom_id: int) -> str:
    name = mujoco_module.mj_id2name(model, mujoco_module.mjtObj.mjOBJ_GEOM, int(geom_id))
    return name or f"geom_{geom_id}"


def parse_geom_size_scale(raw_value, *, to_float_array: ArrayParser) -> np.ndarray | None:
    arr = to_float_array(raw_value)
    if arr is None:
        return None
    if arr.size == 1:
        arr = np.repeat(arr, 3)
    if arr.size != 3:
        return None
    if not np.all(np.isfinite(arr)):
        return None
    return np.clip(arr.astype(np.float64, copy=False), 0.02, 10.0)


def fluid_geom_mask(model) -> np.ndarray:
    return (model.geom_fluid[:, 0] > 0.5) & np.any(
        np.abs(model.geom_fluid[:, 1:6]) > 1e-12,
        axis=1,
    )


def matching_geom_ids(
    fluid_geom_names: dict[int, str],
    pattern: object,
    *,
    case_sensitive: bool,
) -> list[int]:
    matcher = fnmatch.fnmatchcase if case_sensitive else fnmatch.fnmatch
    return [geom_id for geom_id, name in fluid_geom_names.items() if matcher(name, str(pattern))]


def parse_space_separated_floats(raw_text: str) -> list[float]:
    try:
        return [float(item) for item in raw_text.replace(",", " ").split()]
    except ValueError:
        return []


__all__ = [
    "ArrayParser",
    "fluid_geom_mask",
    "fluid_geom_name",
    "matching_geom_ids",
    "parse_geom_size_scale",
    "parse_space_separated_floats",
]
