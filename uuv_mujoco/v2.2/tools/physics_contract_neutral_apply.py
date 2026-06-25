"""Apply neutral open-plant buoyancy forces into MuJoCo state."""

from __future__ import annotations

from typing import Any

import numpy as np

from physics_contract_buoyancy import component_share
from physics_contract_neutral_components import component_buoyancy
from physics_contract_neutral_context import NeutralBuoyancyContext
from physics_contract_neutral_restoring import restoring_torque


def apply_neutral_buoyancy(data: Any, context: NeutralBuoyancyContext) -> tuple[float, float]:
    base_origin = data.xpos[context.base_id].copy()
    base_rot = data.xmat[context.base_id].reshape(3, 3)
    com = data.xipos[context.base_id].copy()

    buoy_force = np.zeros(3, dtype=np.float64)
    buoy_tau = np.zeros(3, dtype=np.float64)
    weighted_submerged = 0.0
    for component in context.components:
        force, force_point_world, submerged = component_buoyancy(base_origin, base_rot, component, context)
        buoy_force += force
        weighted_submerged += component_share(component, context.total_share) * submerged
        if abs(context.cob_torque_scale) > 1.0e-9:
            buoy_tau += np.cross(force_point_world - com, force) * context.cob_torque_scale

    buoy_tau += restoring_torque(data, base_rot, context)
    data.xfrc_applied[context.base_id, 0:3] += buoy_force
    data.xfrc_applied[context.base_id, 3:6] += buoy_tau
    return float(buoy_force[2]), float(np.clip(weighted_submerged, 0.0, 1.0))


__all__ = ["apply_neutral_buoyancy"]
