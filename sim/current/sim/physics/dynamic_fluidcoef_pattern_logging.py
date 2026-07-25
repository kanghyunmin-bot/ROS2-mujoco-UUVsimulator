"""Logging helpers for dynamic MuJoCo fluidcoef pattern setup."""

from __future__ import annotations

import numpy as np

from sim.physics.dynamic_fluidcoef_pattern_prepare import DynamicFluidcoefPatternSetup


def log_dynamic_fluidcoef_pattern_loaded(
    *,
    pattern: str,
    pattern_setup: DynamicFluidcoefPatternSetup,
    fluid_geom_names: dict[int, str],
) -> None:
    print(
        "[physics] dynamic MuJoCo fluidcoef target loaded: "
        f"pattern={pattern!r}, count={len(pattern_setup.matching_geom_ids)}, "
        f"geoms={', '.join(fluid_geom_names[geom_id] for geom_id in pattern_setup.matching_geom_ids)}, "
        "target/reference static scale ratio="
        f"{np.array2string(pattern_setup.ratio, precision=3)}, "
        "load_weights="
        f"{np.array2string(pattern_setup.row_weights, precision=3)}, "
        "axis_weights="
        f"{np.array2string(pattern_setup.row_axis_weights, precision=2)}, "
        "angular_axis_weights="
        f"{np.array2string(pattern_setup.row_angular_axis_weights, precision=2)}",
        flush=True,
    )


__all__ = ["log_dynamic_fluidcoef_pattern_loaded"]
