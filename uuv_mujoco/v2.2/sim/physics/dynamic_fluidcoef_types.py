"""Types and default coefficient weights for dynamic MuJoCo fluidcoef."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


DEFAULT_FLUIDCOEF_AXIS_WEIGHTS = np.array(
    [
        [0.0, 1.0, 1.0],  # blunt drag: crossflow
        [1.0, 0.0, 0.0],  # slender drag: long-axis flow
        [0.0, 0.0, 0.0],  # angular drag: angular axis weights below
        [0.0, 1.0, 1.0],  # Kutta lift: crossflow/AoA proxy
        [1.0, 1.0, 1.0],  # Magnus lift: translation component
    ],
    dtype=np.float64,
)

DEFAULT_FLUIDCOEF_ANGULAR_AXIS_WEIGHTS = np.array(
    [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],  # angular drag
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],  # Magnus lift: angular component
    ],
    dtype=np.float64,
)


@dataclass
class DynamicFluidcoefSetup:
    """Prepared dynamic-fluidcoef arrays for the MuJoCo runtime loop."""

    cfg: dict
    enabled: bool
    base: np.ndarray
    current: np.ndarray
    reference: np.ndarray
    weights: np.ndarray
    axis_weights: np.ndarray
    angular_axis_weights: np.ndarray
    active_geom_ids: set[int]
