"""Cable-like component classification for Ping360 STL filtering."""

from __future__ import annotations


def is_cable_like(component: dict, cable_min_length_mm: float, cable_max_thickness_mm: float) -> bool:
    dims = sorted(component["size"], reverse=True)
    return dims[0] >= cable_min_length_mm and dims[1] <= cable_max_thickness_mm


def classify_components(
    components: list[dict],
    *,
    keep_min_z_mm: float,
    cable_min_length_mm: float,
    cable_max_thickness_mm: float,
) -> set[int]:
    keep_roots = set()
    for component in components:
        cable_like = is_cable_like(component, cable_min_length_mm, cable_max_thickness_mm)
        component["cable_like"] = cable_like
        component["kept"] = (not cable_like) and component["max"][2] >= keep_min_z_mm
        if component["kept"]:
            keep_roots.add(component["root"])
    return keep_roots


__all__ = ["classify_components", "is_cable_like"]
