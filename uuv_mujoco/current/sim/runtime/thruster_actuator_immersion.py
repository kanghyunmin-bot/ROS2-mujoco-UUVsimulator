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

    site_position = np.asarray(runtime.data.site_xpos[sid], dtype=np.float64)
    surface_sampler = getattr(runtime, "surface_height_sampler", None)
    if surface_sampler is None:
        surface_height = float(runtime.water_surface_z)
    else:
        surface_height = float(
            surface_sampler(site_position.copy(), float(runtime.data.time))
        )
        if not np.isfinite(surface_height):
            raise ValueError("thruster surface sampler must return a finite height")
    site_depth_m = float(surface_height - site_position[2])
    water_fraction = submerged_fraction(
        site_depth_m,
        runtime.thruster_immersion_half_height_m,
        runtime.buoyancy_model,
    )
    water_fraction = float(np.clip(water_fraction, 0.0, 1.0))
    return float(runtime.thruster_air_force_scale + (1.0 - runtime.thruster_air_force_scale) * water_fraction)


__all__ = ["force_immersion_scale"]
