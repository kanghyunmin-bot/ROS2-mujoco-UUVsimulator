"""Added-mass matrix helpers for Fossen residual hydrodynamics."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from sim.physics.fossen_residual_config import config_group
from sim.physics.fossen_residual_types import FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS


def added_mass_matrix(added_mass_diag: dict[str, float], added_mass_coupling: dict[str, float]) -> np.ndarray:
    matrix = np.zeros((6, 6), dtype=np.float64)
    for idx, key in enumerate(FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS):
        matrix[idx, idx] = float(added_mass_diag[key])
    matrix[1, 5] = matrix[5, 1] = float(added_mass_coupling["y_r_n_v"])
    matrix[2, 4] = matrix[4, 2] = float(added_mass_coupling["z_q_m_w"])
    return matrix


def added_mass_active(
    cfg: dict[str, Any],
    cfg_active: bool,
    matrix: np.ndarray,
    env_flag: Callable[[str, bool], bool],
) -> bool:
    added_mass_cfg = config_group(cfg, "added_mass")
    return (
        cfg_active
        and bool(env_flag("UUV_FOSSEN_RES_ADDED_MASS_ACTIVE", bool(added_mass_cfg.get("active", True))))
        and np.any(np.abs(matrix) > 1.0e-9)
    )


__all__ = ["added_mass_active", "added_mass_matrix"]
