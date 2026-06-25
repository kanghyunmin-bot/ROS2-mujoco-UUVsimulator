"""Ellipsoid-derived baseline hydrodynamics for simulation profiles."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .hydrodynamics_helpers import estimate_ellipsoid_hydrodynamics
from .sim_profile_parsing import to_float_array, vector3_from_keys


def resolve_ellipsoid_baseline(
    sim_profile: Mapping[str, Any],
    fluid_density: float,
) -> tuple[dict[str, Any] | None, str]:
    payload = sim_profile.get("ellipsoid_model")
    if not isinstance(payload, Mapping) or not bool(payload.get("active", False)):
        return None, "coefficient-profile"

    semi_axes = to_float_array(payload.get("semi_axes"), 3)
    if semi_axes is None or np.any(semi_axes <= 0.0):
        return None, "coefficient-profile"

    estimate = estimate_ellipsoid_hydrodynamics(
        semi_axes,
        fluid_density,
        effective_cd_linear=vector3_from_keys(
            payload,
            "effective_cd_linear",
            "quadratic_cd_linear",
            default=(0.10, 0.11, 0.13),
        ),
        effective_cd_angular=vector3_from_keys(
            payload,
            "effective_cd_angular",
            "quadratic_cd_angular",
            default=(2.2, 2.4, 1.8),
        ),
        added_mass_scale_linear=vector3_from_keys(
            payload,
            "added_mass_scale_linear",
            default=(1.0, 1.0, 1.0),
        ),
        added_mass_scale_angular=vector3_from_keys(
            payload,
            "added_mass_scale_angular",
            default=(1.2, 1.2, 1.0),
        ),
        linear_damping_ratio_linear=float(payload.get("linear_damping_ratio_linear", 2.0)),
        linear_damping_ratio_angular=float(payload.get("linear_damping_ratio_angular", 1.0)),
        reference_speed_linear=float(payload.get("reference_speed_linear", 0.30)),
        reference_speed_angular=float(payload.get("reference_speed_angular", 0.75)),
    )
    return {
        "semi_axes": estimate.semi_axes,
        "half_height": float(payload.get("half_height", estimate.semi_axes[2])),
        "displaced_volume": (
            estimate.displaced_volume if bool(payload.get("use_shape_volume", True)) else None
        ),
        "added_mass_diag": estimate.added_mass_diag,
        "linear_damping_diag": estimate.linear_damping_diag,
        "quadratic_damping_diag": estimate.quadratic_damping_diag,
    }, "ellipsoid-baseline"


__all__ = ["resolve_ellipsoid_baseline"]
