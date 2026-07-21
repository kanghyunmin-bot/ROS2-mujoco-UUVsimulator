"""Thruster water-immersion scaling for actuator force runtime."""

from __future__ import annotations

from typing import Any

import numpy as np

from physics.hydrodynamics_helpers import submerged_fraction


def force_immersion_scale(runtime: Any, thr_name: str) -> float:
    """Scale plant force by the actual thruster site water immersion."""

    sid = runtime.site_ids.get(thr_name, -1)
    if sid < 0:
        return 1.0

    site_z = float(runtime.data.site_xpos[sid, 2])
    site_depth_m = float(runtime.water_surface_z - site_z)
    water_fraction = submerged_fraction(
        site_depth_m,
        runtime.thruster_immersion_half_height_m,
        runtime.buoyancy_model,
    )
    water_fraction = float(np.clip(water_fraction, 0.0, 1.0))
    return float(runtime.thruster_air_force_scale + (1.0 - runtime.thruster_air_force_scale) * water_fraction)


__all__ = ["force_immersion_scale"]
