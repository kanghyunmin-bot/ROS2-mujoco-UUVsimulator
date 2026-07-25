"""One-dimensional CFD table interpolation helpers."""

from __future__ import annotations

import numpy as np


def cfd_force_table_lookup(
    speed: float,
    speeds: np.ndarray,
    magnitudes: np.ndarray,
    extrapolate: str,
) -> float:
    """Look up a non-negative force magnitude from a one-dimensional CFD table."""
    speed = float(abs(speed))
    if speed <= 1.0e-12 or speeds.size == 0 or magnitudes.size == 0:
        return 0.0

    speeds = np.asarray(speeds, dtype=np.float64)
    magnitudes = np.asarray(magnitudes, dtype=np.float64)
    order = np.argsort(speeds)
    speeds = speeds[order]
    magnitudes = magnitudes[order]

    if speeds[0] > 1.0e-9:
        speeds = np.concatenate(([0.0], speeds))
        magnitudes = np.concatenate(([0.0], magnitudes))

    if speed <= float(speeds[-1]):
        return float(np.interp(speed, speeds, magnitudes))

    return _extrapolate_force(speed, speeds, magnitudes, extrapolate)


def _extrapolate_force(
    speed: float,
    speeds: np.ndarray,
    magnitudes: np.ndarray,
    extrapolate: str,
) -> float:
    last_speed = max(float(speeds[-1]), 1.0e-9)
    last_mag = float(magnitudes[-1])
    mode = str(extrapolate).strip().lower()
    if mode in {"quadratic", "u2", "square"}:
        return float(last_mag * (speed / last_speed) ** 2)
    if mode in {"hold", "constant", "clamp"}:
        return float(last_mag)
    return float(last_mag * (speed / last_speed))


__all__ = ["cfd_force_table_lookup"]
