"""Hydrostatic restoring torque for neutral open-plant checks."""

from __future__ import annotations

from typing import Any

import numpy as np

from physics_contract_geometry import rpy_rad_from_quat_wxyz
from physics_contract_neutral_context import NeutralBuoyancyContext


def restoring_torque(data: Any, base_rot: np.ndarray, context: NeutralBuoyancyContext) -> np.ndarray:
    if not bool(context.hydro_cfg.hydrostatic_restoring_active):
        return np.zeros(3, dtype=np.float64)
    quat = data.qpos[context.world_qpos_adr + 3 : context.world_qpos_adr + 7].copy()
    roll, pitch, _ = rpy_rad_from_quat_wxyz(quat)
    restoring_tau_body = np.array(
        [
            -float(context.hydro_cfg.hydrostatic_restoring_roll_stiffness) * float(roll),
            -float(context.hydro_cfg.hydrostatic_restoring_pitch_stiffness) * float(pitch),
            0.0,
        ],
        dtype=np.float64,
    )
    return base_rot @ restoring_tau_body


__all__ = ["restoring_torque"]
