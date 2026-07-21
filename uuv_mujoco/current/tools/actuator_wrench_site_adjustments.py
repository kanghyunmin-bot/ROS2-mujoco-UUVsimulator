"""Runtime site-position adjustments for actuator wrench audits."""

from __future__ import annotations

from typing import Any

import numpy as np

from actuator_wrench_common import finite_float
from actuator_wrench_sites import actuator_site_id
from physics.thruster_mapping import PHYSICAL_VERTICAL_THRUSTERS, PHYSICAL_YAW_THRUSTERS


def horizontal_z_offset_from_profile(profile: dict[str, Any], override: float | None) -> float:
    return finite_float(
        override if override is not None else profile.get("horizontal_thruster_z_offset_m"),
        0.0,
    )


def vertical_x_scale_from_profile(profile: dict[str, Any], override: float | None) -> float:
    return finite_float(
        override if override is not None else profile.get("vertical_thruster_x_scale"),
        1.0,
    )


def apply_horizontal_z_offset(model: mujoco.MjModel, offset_m: float) -> None:
    if abs(offset_m) <= 1.0e-12:
        return
    offset_m = float(np.clip(offset_m, -0.2, 0.2))
    for name in PHYSICAL_YAW_THRUSTERS:
        sid = actuator_site_id(model, name)
        if sid >= 0:
            model.site_pos[sid, 2] += offset_m


def apply_vertical_x_scale(model: mujoco.MjModel, scale_value: float) -> None:
    if abs(scale_value - 1.0) <= 1.0e-12:
        return
    sids = [actuator_site_id(model, name) for name in PHYSICAL_VERTICAL_THRUSTERS]
    sids = [sid for sid in sids if sid >= 0]
    if not sids:
        return
    center_x = float(np.mean([model.site_pos[sid, 0] for sid in sids]))
    scale = float(np.clip(scale_value, 0.25, 4.0))
    for name in PHYSICAL_VERTICAL_THRUSTERS:
        sid = actuator_site_id(model, name)
        if sid >= 0:
            old_x = float(model.site_pos[sid, 0])
            model.site_pos[sid, 0] = center_x + (old_x - center_x) * scale


def apply_runtime_site_adjustments(
    model: mujoco.MjModel,
    profile: dict[str, Any],
    *,
    horizontal_z_offset: float | None,
    vertical_x_scale: float | None,
) -> None:
    apply_horizontal_z_offset(
        model,
        horizontal_z_offset_from_profile(profile, horizontal_z_offset),
    )
    apply_vertical_x_scale(
        model,
        vertical_x_scale_from_profile(profile, vertical_x_scale),
    )


__all__ = [
    "apply_horizontal_z_offset",
    "apply_runtime_site_adjustments",
    "apply_vertical_x_scale",
    "horizontal_z_offset_from_profile",
    "vertical_x_scale_from_profile",
]
