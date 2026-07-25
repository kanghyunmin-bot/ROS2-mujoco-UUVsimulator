"""Normalization helpers for hydrostatic simulation profile fields."""

from __future__ import annotations

from typing import Any


def normalize_buoyancy_model(value: Any) -> str:
    name = str(value).strip().lower()
    if name in {"linear", "ellipsoid"}:
        return name
    return "ellipsoid"


def normalize_hydrostatic_volume_source(value: Any) -> str:
    name = str(value).strip().lower().replace("-", "_")
    if name in {"body_component", "body_components", "component", "components"}:
        return "body_components"
    if name in {"buoyancy_point", "buoyancy_points", "point", "points", "legacy_points"}:
        return "buoyancy_points"
    return "auto"


def normalize_component_shape(value: Any) -> str:
    name = str(value).strip().lower().replace("-", "_")
    if name in {"box", "ellipsoid", "cylinder", "capsule"}:
        return name
    if name in {"spheroid", "ellipse"}:
        return "ellipsoid"
    if name in {"cyl", "elliptic_cylinder", "elliptical_cylinder"}:
        return "cylinder"
    if name in {"caps", "spherocylinder", "capsule_x"}:
        return "capsule"
    return "ellipsoid"


__all__ = [
    "normalize_buoyancy_model",
    "normalize_component_shape",
    "normalize_hydrostatic_volume_source",
]
