"""Per-geom runtime scale for MuJoCo geom fluid coefficients."""

from __future__ import annotations

import numpy as np

from sim.physics.fluid_geom_common import ArrayParser, matching_geom_ids


def apply_per_geom_fluidcoef_scales(
    model,
    sim_profile: dict,
    fluid_geom_ids: np.ndarray,
    fluid_geom_names: dict[int, str],
    fluidcoef_static_geom_scales: dict[str, np.ndarray],
    *,
    to_float_array: ArrayParser,
) -> None:
    fluidcoef_geom_scales = sim_profile.get("mujoco_fluidcoef_geom_scales")
    if not isinstance(fluidcoef_geom_scales, dict) or not fluid_geom_ids.size:
        return
    for pattern, raw_scale in fluidcoef_geom_scales.items():
        _apply_one_per_geom_scale(
            model,
            fluid_geom_names=fluid_geom_names,
            fluidcoef_static_geom_scales=fluidcoef_static_geom_scales,
            pattern=pattern,
            raw_scale=raw_scale,
            to_float_array=to_float_array,
        )


def _apply_one_per_geom_scale(
    model,
    *,
    fluid_geom_names: dict[int, str],
    fluidcoef_static_geom_scales: dict[str, np.ndarray],
    pattern: str,
    raw_scale,
    to_float_array: ArrayParser,
) -> None:
    geom_scale = to_float_array(raw_scale)
    if geom_scale is None or geom_scale.size != 5:
        print(
            "[physics] ignoring mujoco_fluidcoef_geom_scales "
            f"for {pattern!r}: expected 5 values "
            "(blunt, slender, angular, Kutta, Magnus)",
            flush=True,
        )
        return
    geom_scale = np.clip(geom_scale.astype(np.float64, copy=False), 0.0, 10.0)
    fluidcoef_static_geom_scales[str(pattern)] = geom_scale.copy()
    matching_ids = matching_geom_ids(fluid_geom_names, pattern, case_sensitive=True)
    if not matching_ids:
        print(
            "[physics] warning: mujoco_fluidcoef_geom_scales pattern "
            f"{pattern!r} matched no fluid geoms",
            flush=True,
        )
        return
    model.geom_fluid[np.array(matching_ids, dtype=np.int32), 1:6] *= geom_scale.reshape(1, 5)
    print(
        "[physics] MuJoCo fluidcoef per-geom scale applied: "
        f"pattern={pattern!r}, count={len(matching_ids)}, "
        f"geoms={', '.join(fluid_geom_names[geom_id] for geom_id in matching_ids)}, "
        "coeff=(blunt, slender, angular, Kutta, Magnus)="
        f"{np.array2string(geom_scale, precision=3)}",
        flush=True,
    )


__all__ = ["apply_per_geom_fluidcoef_scales"]
