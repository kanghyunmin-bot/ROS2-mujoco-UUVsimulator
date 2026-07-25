"""Debug logging helpers for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from typing import Any

import numpy as np


def log_dynamic_fluidcoef_update(
    runtime: Any,
    *,
    sim_t: float,
    idx: np.ndarray,
    first_loads: np.ndarray,
    transient: np.ndarray,
) -> None:
    if not runtime.debug or sim_t - runtime.last_log_sim_t < 2.0:
        return
    runtime.last_log_sim_t = sim_t
    first_id = int(idx[0])
    ratio = np.divide(
        runtime.current[first_id, :],
        np.maximum(runtime.base[first_id, :], 1.0e-12),
    )
    print(
        "[physics] dynamic fluidcoef update: "
        f"geom={runtime.fluid_geom_names.get(first_id, first_id)!r}, "
        f"loads={np.array2string(first_loads, precision=3)}, "
        f"transient={np.array2string(transient[0, :], precision=3)}, "
        f"ratio={np.array2string(ratio, precision=3)}",
        flush=True,
    )


__all__ = ["log_dynamic_fluidcoef_update"]
