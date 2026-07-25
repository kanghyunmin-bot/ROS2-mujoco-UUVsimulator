"""Vertical thruster x-lever scaling primitives."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np

from sim.physics.actuator_geometry_sites import actuator_site_id


def vertical_thruster_site_ids(
    *,
    model: Any,
    mujoco_module: Any,
    actuator_ids: Mapping[str, int],
    thruster_names: Sequence[str],
) -> list[int]:
    site_ids = [
        actuator_site_id(
            model=model,
            mujoco_module=mujoco_module,
            actuator_ids=actuator_ids,
            name=name,
        )
        for name in thruster_names
    ]
    return [site_id for site_id in site_ids if site_id >= 0]


def vertical_scale_center_x(model: Any, *, site_ids: Sequence[int], center_x: float | None) -> float:
    if center_x is None:
        return float(np.mean([model.site_pos[site_id, 0] for site_id in site_ids]))
    return float(center_x)


def clipped_vertical_x_scale(scale_raw: float) -> float:
    return float(np.clip(float(scale_raw), 0.25, 4.0))


def apply_vertical_x_scale_to_sites(
    *,
    model: Any,
    mujoco_module: Any,
    actuator_ids: Mapping[str, int],
    thruster_names: Sequence[str],
    center: float,
    scale: float,
) -> list[str]:
    adjusted = []
    for name in thruster_names:
        site_id = actuator_site_id(
            model=model,
            mujoco_module=mujoco_module,
            actuator_ids=actuator_ids,
            name=name,
        )
        if site_id < 0:
            continue
        old_x = float(model.site_pos[site_id, 0])
        model.site_pos[site_id, 0] = center + (old_x - center) * scale
        adjusted.append(f"{name}:{old_x:.4f}->{float(model.site_pos[site_id, 0]):.4f}")
    return adjusted


__all__ = [
    "apply_vertical_x_scale_to_sites",
    "clipped_vertical_x_scale",
    "vertical_scale_center_x",
    "vertical_thruster_site_ids",
]
