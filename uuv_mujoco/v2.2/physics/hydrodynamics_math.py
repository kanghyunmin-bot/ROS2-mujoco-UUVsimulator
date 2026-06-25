"""Core 6DOF hydrodynamics math helpers."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def skew(vec: Iterable[float]) -> np.ndarray:
    x, y, z = np.asarray(list(vec), dtype=np.float64)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


def added_mass_coriolis(added_mass: np.ndarray, nu_body: np.ndarray) -> np.ndarray:
    """Return an added-mass Coriolis matrix in Fossen ordering."""

    raw = np.asarray(added_mass, dtype=np.float64)
    if raw.shape == (6, 6):
        mass_matrix = raw
    else:
        mass_matrix = np.diag(raw.reshape(6))
    nu = np.asarray(nu_body, dtype=np.float64).reshape(6)
    linear = nu[:3]
    angular = nu[3:]
    a_term = mass_matrix[:3, :3] @ linear + mass_matrix[:3, 3:] @ angular
    b_term = mass_matrix[3:, :3] @ linear + mass_matrix[3:, 3:] @ angular
    coriolis = np.zeros((6, 6), dtype=np.float64)
    coriolis[:3, 3:] = -skew(a_term)
    coriolis[3:, :3] = -skew(a_term)
    coriolis[3:, 3:] = -skew(b_term)
    return coriolis


def first_order_response(current: float, target: float, dt: float, tau_up: float, tau_down: float) -> float:
    if dt <= 0.0:
        return current
    tau = tau_up if abs(target) >= abs(current) else tau_down
    tau = max(float(tau), 1e-6)
    alpha = 1.0 - math.exp(-dt / tau)
    return float(current + alpha * (target - current))


__all__ = ["added_mass_coriolis", "first_order_response", "skew"]
