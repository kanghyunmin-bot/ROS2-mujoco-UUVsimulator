"""Composite mass and inertia calculations for body components."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os

import numpy as np


@dataclass
class CompositeBodyDistribution:
    total_mass: float
    composite_com: np.ndarray
    composite_inertia: np.ndarray
    inertia_scale: np.ndarray


def component_self_inertia_diag(component) -> np.ndarray:
    """Return a diagonal self-inertia estimate for one profile body component."""
    a, b, c = component.size.astype(np.float64, copy=False)
    if component.shape == "box":
        return np.array(
            [
                component.mass * (b * b + c * c) / 3.0,
                component.mass * (a * a + c * c) / 3.0,
                component.mass * (a * a + b * b) / 3.0,
            ],
            dtype=np.float64,
        )
    if component.shape == "cylinder":
        return cylinder_self_inertia_diag(component.mass, half_length=float(a), radius_y=float(b), radius_z=float(c))
    if component.shape == "capsule":
        return capsule_self_inertia_diag(component.mass, half_length=float(a), radius_y=float(b), radius_z=float(c))
    return np.array(
        [
            component.mass * (b * b + c * c) / 5.0,
            component.mass * (a * a + c * c) / 5.0,
            component.mass * (a * a + b * b) / 5.0,
        ],
        dtype=np.float64,
    )


def cylinder_self_inertia_diag(
    mass: float,
    *,
    half_length: float,
    radius_y: float,
    radius_z: float,
) -> np.ndarray:
    """Solid elliptical cylinder inertia, with its length along the body x-axis."""

    a = max(float(half_length), 1.0e-9)
    b = max(float(radius_y), 1.0e-9)
    c = max(float(radius_z), 1.0e-9)
    m = max(float(mass), 0.0)
    return np.array(
        [
            m * (b * b + c * c) / 4.0,
            m * (a * a / 3.0 + c * c / 4.0),
            m * (a * a / 3.0 + b * b / 4.0),
        ],
        dtype=np.float64,
    )


def capsule_self_inertia_diag(
    mass: float,
    *,
    half_length: float,
    radius_y: float,
    radius_z: float,
) -> np.ndarray:
    """Approximate x-axis capsule inertia from a cylinder plus two ellipsoid caps."""

    a = max(float(half_length), 1.0e-9)
    b = max(float(radius_y), 1.0e-9)
    c = max(float(radius_z), 1.0e-9)
    cap_x_radius = 0.5 * (b + c)
    cyl_half = max(a - cap_x_radius, 0.0)
    cyl_volume = math.pi * b * c * (2.0 * cyl_half)
    cap_volume = (4.0 / 3.0) * math.pi * cap_x_radius * b * c
    total_volume = max(cyl_volume + cap_volume, 1.0e-12)
    m = max(float(mass), 0.0)
    cyl_mass = m * cyl_volume / total_volume
    cap_mass = m - cyl_mass
    inertia = cylinder_self_inertia_diag(cyl_mass, half_length=cyl_half, radius_y=b, radius_z=c)
    cap_self = np.array(
        [
            cap_mass * (b * b + c * c) / 5.0,
            cap_mass * (cap_x_radius * cap_x_radius + c * c) / 5.0,
            cap_mass * (cap_x_radius * cap_x_radius + b * b) / 5.0,
        ],
        dtype=np.float64,
    )
    cap_offset = cyl_half
    cap_parallel = cap_mass * np.array([0.0, cap_offset * cap_offset, cap_offset * cap_offset], dtype=np.float64)
    return inertia + cap_self + cap_parallel


def compute_composite_body_distribution(components, sim_profile: dict) -> CompositeBodyDistribution | None:
    if not components:
        return None
    total_mass = float(sum(component.mass for component in components))
    if total_mass <= 1e-9:
        return None
    inertia_scale = body_inertia_scale(sim_profile)
    composite_com = sum(component.mass * component.mass_pos for component in components) / total_mass
    composite_inertia = np.zeros(3, dtype=np.float64)
    for component in components:
        composite_inertia += component_self_inertia_diag(component) + parallel_axis_inertia(component, composite_com)
    composite_inertia *= inertia_scale
    return CompositeBodyDistribution(
        total_mass=total_mass,
        composite_com=composite_com,
        composite_inertia=composite_inertia,
        inertia_scale=inertia_scale,
    )


def body_inertia_scale(sim_profile: dict) -> np.ndarray:
    inertia_scale = np.array(
        sim_profile.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]),
        dtype=np.float64,
    )
    if inertia_scale.shape != (3,) or not np.all(np.isfinite(inertia_scale)):
        inertia_scale = np.ones(3, dtype=np.float64)
    env_scale = _body_inertia_scale_from_env(inertia_scale)
    if env_scale is not None:
        inertia_scale = env_scale
    return np.clip(inertia_scale, 1e-6, 100.0)


def _body_inertia_scale_from_env(profile_scale: np.ndarray) -> np.ndarray | None:
    raw_xyz = os.environ.get("UUV_BODY_INERTIA_SCALE_XYZ", "").strip()
    if raw_xyz:
        try:
            values = [float(part) for part in raw_xyz.replace(",", " ").split()]
        except ValueError:
            values = []
        if len(values) == 3 and all(np.isfinite(values)):
            return np.array(values, dtype=np.float64)
    axis_values: list[float | None] = []
    for axis in ("X", "Y", "Z"):
        raw_value = os.environ.get(f"UUV_BODY_INERTIA_SCALE_{axis}", "").strip()
        if not raw_value:
            axis_values.append(None)
            continue
        try:
            value = float(raw_value)
        except ValueError:
            axis_values.append(None)
            continue
        axis_values.append(value if np.isfinite(value) else None)
    if all(value is None for value in axis_values):
        return None
    base = np.array(profile_scale, dtype=np.float64, copy=True)
    for idx, value in enumerate(axis_values):
        if value is not None:
            base[idx] = float(value)
    return base


def parallel_axis_inertia(component, composite_com: np.ndarray) -> np.ndarray:
    offset = component.mass_pos - composite_com
    return component.mass * np.array(
        [
            offset[1] * offset[1] + offset[2] * offset[2],
            offset[0] * offset[0] + offset[2] * offset[2],
            offset[0] * offset[0] + offset[1] * offset[1],
        ],
        dtype=np.float64,
    )


__all__ = [
    "CompositeBodyDistribution",
    "body_inertia_scale",
    "capsule_self_inertia_diag",
    "component_self_inertia_diag",
    "compute_composite_body_distribution",
    "cylinder_self_inertia_diag",
    "parallel_axis_inertia",
]
