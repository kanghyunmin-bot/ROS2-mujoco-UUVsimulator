"""Fluid-geom pattern matching for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

import fnmatch


def matching_fluid_geom_ids(fluid_geom_names: dict[int, str], pattern: str) -> list[int]:
    return [
        int(geom_id)
        for geom_id, geom_name in fluid_geom_names.items()
        if fnmatch.fnmatchcase(geom_name, str(pattern))
    ]


__all__ = ["matching_fluid_geom_ids"]
