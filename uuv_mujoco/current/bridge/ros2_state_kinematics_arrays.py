"""Array validation helpers for MuJoCo kinematic reads."""

from __future__ import annotations

import numpy as np


def finite_vector_copy(value, *, min_size: int | None = None) -> np.ndarray | None:
    try:
        arr = np.array(value, dtype=np.float64)
    except Exception:
        return None
    if min_size is not None and arr.size < min_size:
        return None
    if not np.all(np.isfinite(arr)):
        return None
    return arr.copy()


def zero_velocity_enu() -> np.ndarray:
    return np.zeros(3, dtype=np.float64)


__all__ = ["finite_vector_copy", "zero_velocity_enu"]
