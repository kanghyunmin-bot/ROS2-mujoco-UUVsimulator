"""Vector reads for real-start velocity extraction."""

from __future__ import annotations

import math

from real_start_common import finite


def finite_row_vector(row: dict[str, str], keys: tuple[str, str, str]) -> tuple[float, float, float] | None:
    values = tuple(finite(row.get(key)) for key in keys)
    if all(math.isfinite(v) for v in values):
        return float(values[0]), float(values[1]), float(values[2])
    return None


def finite_row_quat_xyzw(row: dict[str, str]) -> tuple[float, float, float, float] | None:
    values = tuple(finite(row.get(key)) for key in ("local_pose_qx", "local_pose_qy", "local_pose_qz", "local_pose_qw"))
    if all(math.isfinite(v) for v in values):
        return float(values[0]), float(values[1]), float(values[2]), float(values[3])
    return None


__all__ = ["finite_row_quat_xyzw", "finite_row_vector"]
