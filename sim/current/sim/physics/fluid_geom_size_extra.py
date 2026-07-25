"""Environment-driven runtime scaling for MuJoCo fluid geom sizes."""

from __future__ import annotations

import numpy as np

from sim.physics.fluid_geom_common import ArrayParser, parse_geom_size_scale, parse_space_separated_floats


def apply_extra_geom_size_scale(
    model,
    fluid_geom_ids: np.ndarray,
    *,
    raw_text: str,
    to_float_array: ArrayParser,
) -> None:
    if not raw_text or not fluid_geom_ids.size:
        return
    geom_size_extra = parse_geom_size_scale(parse_space_separated_floats(raw_text), to_float_array=to_float_array)
    if geom_size_extra is None:
        print(
            "[physics] ignoring UUV_MJ_FLUID_GEOM_SIZE_EXTRA_SCALE="
            f"{raw_text!r}; expected scalar or 3 finite values",
            flush=True,
        )
        return
    model.geom_size[fluid_geom_ids, 0:3] *= geom_size_extra.reshape(1, 3)
    print(
        "[physics] MuJoCo fluid geom size extra runtime scale applied: "
        f"count={int(fluid_geom_ids.size)}, "
        f"scale_xyz={np.array2string(geom_size_extra, precision=3)}",
        flush=True,
    )


__all__ = ["apply_extra_geom_size_scale"]
