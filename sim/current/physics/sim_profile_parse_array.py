"""Array parsers for simulation profile fields."""

from __future__ import annotations

from typing import Any

import numpy as np


def to_float_array(value: Any, size: int) -> np.ndarray | None:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        return None
    try:
        out = np.array([float(item) for item in value], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(out)):
        return None
    return out


__all__ = ["to_float_array"]
