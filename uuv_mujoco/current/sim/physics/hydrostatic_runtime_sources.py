"""Hydrostatic volume source selection."""

from __future__ import annotations

from typing import Any


def resolve_hydrostatic_sources(hydro_cfg: Any) -> tuple[tuple[Any, ...], tuple[Any, ...], str]:
    source = str(hydro_cfg.hydrostatic_volume_source)
    components = tuple(hydro_cfg.body_components)
    points = tuple(hydro_cfg.buoyancy_points)
    if source == "body_components":
        if components:
            return components, (), source
        return (), points, "buoyancy_points-fallback"
    if source == "buoyancy_points":
        if points:
            return (), points, source
        return components, (), "body_components-fallback"
    if components:
        return components, (), "body_components-auto"
    if points:
        return (), points, "buoyancy_points-auto"
    return (), (), "single_cob"


__all__ = ["resolve_hydrostatic_sources"]
