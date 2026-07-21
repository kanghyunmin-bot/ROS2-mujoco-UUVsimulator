"""Per-step update helpers for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.physics.dynamic_fluidcoef_loads import fluidcoef_loads_from_local_velocity
from sim.physics.dynamic_fluidcoef_runtime_logging import log_dynamic_fluidcoef_update
from sim.physics.dynamic_fluidcoef_runtime_transient import (
    apply_dynamic_fluidcoef_transient,
    dynamic_fluidcoef_decay,
)


def dynamic_fluidcoef_update_due(runtime: Any, sim_t: float) -> bool:
    if runtime.next_sim_t < 0.0:
        runtime.next_sim_t = sim_t
    if sim_t + 1.0e-9 < runtime.next_sim_t:
        return False
    while sim_t + 1.0e-9 >= runtime.next_sim_t:
        runtime.next_sim_t += runtime.update_dt
    return True


def compute_dynamic_fluidcoef_blend(
    runtime: Any,
    idx: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    blend = np.zeros((idx.size, 5), dtype=np.float64)
    first_loads = np.zeros(5, dtype=np.float64)
    for row, geom_id in enumerate(idx):
        vel6 = np.zeros(6, dtype=np.float64)
        runtime.mujoco.mj_objectVelocity(
            runtime.model,
            runtime.data,
            runtime.mujoco.mjtObj.mjOBJ_GEOM,
            int(geom_id),
            vel6,
            1,
        )
        geom_rot = runtime.data.geom_xmat[int(geom_id)].reshape(3, 3)
        current_local = geom_rot.T @ runtime.water_current_world
        coeff_loads = fluidcoef_loads_from_local_velocity(
            vel6[3:] - current_local,
            vel6[:3],
            runtime.axis_weights[int(geom_id), :, :],
            runtime.angular_axis_weights[int(geom_id), :, :],
            reference_speed_mps=runtime.ref_speed,
            reference_angular_rps=runtime.ref_angular,
        )
        if row == 0:
            first_loads = coeff_loads
        blend[row, :] = np.clip(runtime.weights[int(geom_id), :] * coeff_loads, 0.0, 1.0)
    return blend, first_loads


def apply_dynamic_fluidcoef_target(
    runtime: Any,
    idx: np.ndarray,
    blend: np.ndarray,
    transient: np.ndarray,
    reset_mask: np.ndarray,
) -> None:
    target = runtime.base[idx, :] + (blend * transient) * (
        runtime.reference[idx, :] - runtime.base[idx, :]
    )
    alpha = np.full_like(target, runtime.alpha, dtype=np.float64)
    alpha[:, ~runtime.log_decay_mask] = runtime.lift_alpha
    if runtime.transient_enabled:
        alpha[reset_mask] = runtime.transient_attack_alpha
    runtime.current[idx, :] += alpha * (target - runtime.current[idx, :])
    runtime.model.geom_fluid[idx, 1:6] = runtime.current[idx, :]


__all__ = [
    "apply_dynamic_fluidcoef_target",
    "apply_dynamic_fluidcoef_transient",
    "compute_dynamic_fluidcoef_blend",
    "dynamic_fluidcoef_decay",
    "dynamic_fluidcoef_update_due",
    "log_dynamic_fluidcoef_update",
]
