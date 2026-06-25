"""Shared parsing and numeric helpers for SITL replay CSVs."""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np


def csv_float(row: dict[str, str], name: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(name, "") or default)
    except (TypeError, ValueError):
        return float(default)
    return float(value) if np.isfinite(value) else float(default)


def normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm <= 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if q[0] < 0.0:
        q = -q
    return q / norm


def pressure_abs_from_depth_m(
    depth_m: float,
    surface_pressure_pa: float,
    rho: float,
    gravity: float,
) -> float:
    depth = float(max(0.0, depth_m))
    return float(surface_pressure_pa + rho * gravity * depth)


def replay_log(log: Optional[Callable[[str], None]], message: str) -> None:
    if log is not None:
        log(message)


__all__ = ["csv_float", "normalize_quat_wxyz", "pressure_abs_from_depth_m", "replay_log"]
