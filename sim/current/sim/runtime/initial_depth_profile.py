"""Profile parsing helpers for initial-depth calculations."""

from __future__ import annotations

import numpy as np


def vec3_from_profile(value) -> np.ndarray | None:
    """Parse a finite 3-vector from a sim profile field."""
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        out = np.array([float(item) for item in value], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(out)):
        return None
    return out


__all__ = ["vec3_from_profile"]
