"""Equivalent-ellipsoid geometry helpers."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def positive_semi_axes(semi_axes: Iterable[float]) -> np.ndarray:
    semi = np.asarray(list(semi_axes), dtype=np.float64).reshape(3)
    if np.any(semi <= 0.0):
        raise ValueError("semi_axes must be positive")
    return semi


def ellipsoid_volume(semi_axes: Iterable[float]) -> float:
    a, b, c = positive_semi_axes(semi_axes)
    return float((4.0 / 3.0) * math.pi * a * b * c)


def ellipsoid_projected_areas(semi_axes: Iterable[float]) -> np.ndarray:
    a, b, c = positive_semi_axes(semi_axes)
    return np.array(
        [
            math.pi * b * c,
            math.pi * a * c,
            math.pi * a * b,
        ],
        dtype=np.float64,
    )


def ellipsoid_depolarization_factors(semi_axes: Iterable[float], samples: int = 4096) -> np.ndarray:
    """Approximate triaxial-ellipsoid depolarization factors by quadrature."""

    a, b, c = positive_semi_axes(semi_axes)
    samples = int(max(samples, 512))
    t = np.linspace(0.0, 1.0 - 1e-7, samples, dtype=np.float64)
    s = t / np.maximum(1.0 - t, 1e-12)
    jac = 1.0 / np.maximum((1.0 - t) ** 2, 1e-12)
    a2, b2, c2 = a * a, b * b, c * c
    root = np.sqrt((s + a2) * (s + b2) * (s + c2))
    prefactor = (a * b * c) / 2.0
    integrate = getattr(np, "trapezoid", None)
    if integrate is None:
        integrate = np.trapz
    factors = []
    for axis_sq in (a2, b2, c2):
        integrand = prefactor / ((s + axis_sq) * root)
        factors.append(float(integrate(integrand * jac, t)))
    out = np.clip(np.array(factors, dtype=np.float64), 1e-9, 1.0)
    total = float(np.sum(out))
    if total <= 1e-9:
        return np.full(3, 1.0 / 3.0, dtype=np.float64)
    return out / total


__all__ = [
    "ellipsoid_depolarization_factors",
    "ellipsoid_projected_areas",
    "ellipsoid_volume",
    "positive_semi_axes",
]
