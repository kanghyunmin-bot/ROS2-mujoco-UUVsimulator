"""Small parsing helpers shared by runtime contract builders."""

from __future__ import annotations

import numpy as np


def to_float_array(values) -> np.ndarray | None:
    if not isinstance(values, list) or not values:
        return None
    out = []
    for value in values:
        try:
            out.append(float(value))
        except (TypeError, ValueError):
            return None
    return np.array(out, dtype=np.float64)


def to_float_matrix(values, shape: tuple[int, int]) -> np.ndarray | None:
    if not isinstance(values, list) or len(values) != shape[0]:
        return None
    rows = []
    for row in values:
        arr = to_float_array(row)
        if arr is None or arr.size != shape[1]:
            return None
        rows.append(arr)
    return np.vstack(rows).astype(np.float64, copy=False)
