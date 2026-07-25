"""Runner startup thruster immersion contract values."""

from __future__ import annotations

import numpy as np


def resolve_thruster_immersion_contract(env_float) -> tuple[float, float]:
    air_force_scale = float(np.clip(env_float("UUV_THRUSTER_AIR_FORCE_SCALE", 0.0), 0.0, 1.0))
    immersion_half_height_m = float(
        max(env_float("UUV_THRUSTER_IMMERSION_HALF_HEIGHT_M", 0.045), 1.0e-4)
    )
    return air_force_scale, immersion_half_height_m


__all__ = ["resolve_thruster_immersion_contract"]
