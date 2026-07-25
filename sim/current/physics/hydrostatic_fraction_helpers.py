"""Waterline submerged-fraction helper models."""

from __future__ import annotations

import numpy as np


def submerged_fraction_linear(depth: float, half_height: float) -> float:
    """Return linear immersed fraction for a box-like hydrostatic proxy."""

    half_height = max(float(half_height), 1e-6)
    return float(np.clip((float(depth) + half_height) / (2.0 * half_height), 0.0, 1.0))


def submerged_fraction_ellipsoid(depth: float, half_height: float) -> float:
    """Return immersed fraction for an ellipsoid cut by a horizontal water plane."""

    half_height = max(float(half_height), 1e-6)
    normalized_depth = float(np.clip(float(depth) / half_height, -1.0, 1.0))
    frac = 0.5 + 0.75 * normalized_depth - 0.25 * (normalized_depth ** 3)
    return float(np.clip(frac, 0.0, 1.0))


def submerged_fraction(depth: float, half_height: float, model: str = "ellipsoid") -> float:
    """Return immersed fraction using the selected hydrostatic proxy model."""

    name = str(model).strip().lower()
    if name == "linear":
        return submerged_fraction_linear(depth, half_height)
    return submerged_fraction_ellipsoid(depth, half_height)


__all__ = ["submerged_fraction", "submerged_fraction_ellipsoid", "submerged_fraction_linear"]
