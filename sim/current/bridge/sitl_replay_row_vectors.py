"""Small vector helpers for SITL replay CSV row parsers."""

from __future__ import annotations

import numpy as np

from bridge.sitl_replay_common import csv_float


def row_vector3(
    row: dict[str, str],
    keys: tuple[str, str, str],
    defaults: tuple[float, float, float],
) -> np.ndarray:
    return np.array(
        [
            csv_float(row, keys[0], defaults[0]),
            csv_float(row, keys[1], defaults[1]),
            csv_float(row, keys[2], defaults[2]),
        ],
        dtype=np.float64,
    )


__all__ = ["row_vector3"]
