"""Math helpers for viewer-only debug drawing."""

from __future__ import annotations

from typing import Any

import numpy as np


def normalize_vector(vector: Any) -> np.ndarray:
    """Return a stable unit vector for viewer-only geometry drawing."""

    arr = np.array(vector, dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-9:
        return np.zeros_like(arr)
    return arr / norm


__all__ = ["normalize_vector"]
