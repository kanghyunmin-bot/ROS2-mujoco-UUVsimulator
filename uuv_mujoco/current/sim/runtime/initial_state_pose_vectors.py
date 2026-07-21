"""Vector parsing helpers for initial pose overrides."""

from __future__ import annotations

from typing import Any

import numpy as np


def finite_vector_or_none(value: Any, shape: tuple[int, ...]) -> np.ndarray | None:
    if value is None:
        return None
    parsed = np.asarray(value, dtype=np.float64)
    if parsed.shape != shape or not np.all(np.isfinite(parsed)):
        return None
    return parsed


def initial_xy_vector(args: Any) -> np.ndarray | None:
    return finite_vector_or_none(args.initial_position_xy, (2,))


def initial_rpy_vector(args: Any) -> np.ndarray | None:
    return finite_vector_or_none(args.initial_rpy_rad, (3,))


__all__ = ["finite_vector_or_none", "initial_rpy_vector", "initial_xy_vector"]
