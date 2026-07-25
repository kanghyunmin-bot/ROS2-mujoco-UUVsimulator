"""Runtime extra scale for MuJoCo geom fluid coefficients."""

from __future__ import annotations

import numpy as np

from sim.physics.fluid_geom_common import parse_space_separated_floats


def apply_extra_fluidcoef_scale(
    model,
    fluid_geom_mask: np.ndarray,
    fluid_geom_ids: np.ndarray,
    *,
    raw_text: str,
) -> None:
    if not raw_text or not fluid_geom_ids.size:
        return
    fluidcoef_extra_scale = np.array(parse_space_separated_floats(raw_text), dtype=np.float64)
    if fluidcoef_extra_scale.size != 5 or not np.all(np.isfinite(fluidcoef_extra_scale)):
        print(
            "[physics] ignoring invalid UUV_MJ_FLUIDCOEF_EXTRA_SCALE="
            f"{raw_text!r}; expected 5 finite values",
            flush=True,
        )
        return
    fluidcoef_extra_scale = np.clip(fluidcoef_extra_scale, 0.0, 10.0)
    model.geom_fluid[fluid_geom_mask, 1:6] *= fluidcoef_extra_scale.reshape(1, 5)
    print(
        "[physics] MuJoCo fluidcoef extra runtime scale applied: "
        f"count={int(fluid_geom_ids.size)}, "
        "coeff=(blunt, slender, angular, Kutta, Magnus)="
        f"{np.array2string(fluidcoef_extra_scale, precision=3)}",
        flush=True,
    )


__all__ = ["apply_extra_fluidcoef_scale"]
