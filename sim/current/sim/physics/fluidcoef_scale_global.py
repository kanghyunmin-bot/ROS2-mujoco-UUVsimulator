"""Runtime global scale for MuJoCo geom fluid coefficients."""

from __future__ import annotations

import numpy as np


def apply_global_fluidcoef_scale(
    model,
    fluid_geom_mask: np.ndarray,
    fluid_geom_ids: np.ndarray,
    fluidcoef_scale: np.ndarray | None,
) -> None:
    if fluidcoef_scale is None:
        return
    if fluidcoef_scale.size != 5:
        print(
            "[physics] ignoring mujoco_fluidcoef_scale: expected 5 values "
            "(blunt, slender, angular, Kutta, Magnus)",
            flush=True,
        )
        return
    fluidcoef_scale = np.clip(fluidcoef_scale.astype(np.float64, copy=False), 0.0, 10.0)
    if not fluid_geom_ids.size:
        return
    model.geom_fluid[fluid_geom_mask, 1:6] *= fluidcoef_scale.reshape(1, 5)
    print(
        "[physics] MuJoCo fluidcoef scale applied: "
        f"count={int(fluid_geom_ids.size)}, "
        "coeff=(blunt, slender, angular, Kutta, Magnus)="
        f"{np.array2string(fluidcoef_scale, precision=3)}",
        flush=True,
    )


__all__ = ["apply_global_fluidcoef_scale"]
