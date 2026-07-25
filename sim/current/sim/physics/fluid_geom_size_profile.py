"""Profile-driven runtime scaling for MuJoCo fluid geom sizes."""

from __future__ import annotations

import numpy as np

from sim.physics.fluid_geom_common import ArrayParser, matching_geom_ids, parse_geom_size_scale


def apply_profile_geom_size_scales(
    model,
    sim_profile: dict,
    fluid_geom_ids: np.ndarray,
    fluid_geom_names: dict[int, str],
    *,
    to_float_array: ArrayParser,
) -> None:
    fluid_geom_size_scales = sim_profile.get("mujoco_fluid_geom_size_scales")
    if not isinstance(fluid_geom_size_scales, dict) or not fluid_geom_ids.size:
        return
    for pattern, raw_scale in fluid_geom_size_scales.items():
        geom_size_scale = parse_geom_size_scale(raw_scale, to_float_array=to_float_array)
        if geom_size_scale is None:
            print(
                "[physics] ignoring mujoco_fluid_geom_size_scales "
                f"for {pattern!r}: expected scalar or 3 values (x, y, z)",
                flush=True,
            )
            continue
        matching_ids = matching_geom_ids(fluid_geom_names, pattern, case_sensitive=False)
        if not matching_ids:
            print(
                "[physics] warning: mujoco_fluid_geom_size_scales pattern "
                f"{pattern!r} matched no fluid geoms",
                flush=True,
            )
            continue
        idx = np.array(matching_ids, dtype=np.int32)
        model.geom_size[idx, 0:3] *= geom_size_scale.reshape(1, 3)
        print(
            "[physics] MuJoCo fluid geom size scale applied: "
            f"pattern={pattern!r}, count={len(matching_ids)}, "
            f"scale_xyz={np.array2string(geom_size_scale, precision=3)}",
            flush=True,
        )


__all__ = ["apply_profile_geom_size_scales"]
