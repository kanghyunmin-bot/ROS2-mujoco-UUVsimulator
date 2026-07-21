"""Custom hydrodynamics dispatch for the underwater wrench runtime."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_hydrodynamics_custom import apply_custom_hydrodynamics


def apply_or_clear_custom_hydrodynamics(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    lin_vel_body: np.ndarray,
    rel_lin_vel_world: np.ndarray,
    nu_rel_body: np.ndarray,
    rel_acc_body: np.ndarray,
    submerged: float,
) -> None:
    if runtime.use_custom_hydrodynamics:
        apply_custom_hydrodynamics(
            runtime,
            base_rot=base_rot,
            lin_vel_body=lin_vel_body,
            rel_lin_vel_world=rel_lin_vel_world,
            nu_rel_body=nu_rel_body,
            rel_acc_body=rel_acc_body,
            submerged=submerged,
        )
        return
    runtime.last_flow_world = np.zeros(3, dtype=np.float64)


__all__ = ["apply_or_clear_custom_hydrodynamics"]
