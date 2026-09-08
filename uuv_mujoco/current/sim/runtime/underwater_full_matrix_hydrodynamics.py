"""Runtime application of opt-in full 6x6 Fossen hydrodynamics."""

from __future__ import annotations

from typing import Any

import numpy as np


def apply_full_matrix_hydrodynamics(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    nu_rel_body: np.ndarray,
    rel_acc_body: np.ndarray,
    submerged: float,
) -> None:
    """Apply the matrix-model body wrench, scaled by immersed fraction."""

    model = getattr(runtime.hydrodynamics, "full_matrix_hydrodynamics", None)
    if model is None or not model.active:
        if hasattr(runtime, "last_full_matrix_wrench_body"):
            runtime.last_full_matrix_wrench_body[:] = 0.0
        return
    # Matrix coefficients and generalized velocity are explicitly referenced
    # to the MuJoCo inertial centre, matching xfrc_applied's torque convention.
    wrench_body = model.wrench_body(nu_rel_body, rel_acc_body) * float(
        np.clip(submerged, 0.0, 1.0)
    )
    if not np.all(np.isfinite(wrench_body)):
        raise FloatingPointError("full-matrix runtime wrench is non-finite")
    base_id = int(runtime.base_id)
    runtime.data.xfrc_applied[base_id, 0:3] += base_rot @ wrench_body[:3]
    runtime.data.xfrc_applied[base_id, 3:6] += base_rot @ wrench_body[3:]
    if hasattr(runtime, "last_full_matrix_wrench_body"):
        runtime.last_full_matrix_wrench_body[:] = wrench_body


__all__ = ["apply_full_matrix_hydrodynamics"]
