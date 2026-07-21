"""State assignment helpers for dynamic MuJoCo fluid coefficient runtime."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef_types import DynamicFluidcoefSetup


def attach_dynamic_fluidcoef_setup_state(
    runtime,
    *,
    setup: DynamicFluidcoefSetup,
    water_current_world: np.ndarray,
    fluid_geom_names: dict[int, str],
) -> None:
    runtime.cfg = setup.cfg
    runtime.enabled = bool(setup.enabled)
    runtime.base = setup.base
    runtime.current = setup.current
    runtime.reference = setup.reference
    runtime.weights = setup.weights
    runtime.axis_weights = setup.axis_weights
    runtime.angular_axis_weights = setup.angular_axis_weights
    runtime.active_idx = np.array(sorted(setup.active_geom_ids), dtype=np.int32)
    runtime.water_current_world = np.asarray(water_current_world, dtype=np.float64)
    runtime.fluid_geom_names = fluid_geom_names


def initialize_dynamic_fluidcoef_runtime_buffers(
    runtime,
    *,
    env_flag: Callable[[str, bool], bool],
) -> None:
    runtime.transient_age_s = np.full_like(runtime.base, np.inf, dtype=np.float64)
    runtime.transient_active = np.zeros_like(runtime.base, dtype=bool)
    runtime.prev_blend = np.zeros_like(runtime.base, dtype=np.float64)
    runtime.next_sim_t = -1.0
    runtime.last_log_sim_t = -10.0
    runtime.debug = env_flag(
        "UUV_DYNAMIC_FLUIDCOEF_DEBUG",
        bool(runtime.cfg.get("debug", False)),
    )


__all__ = [
    "attach_dynamic_fluidcoef_setup_state",
    "initialize_dynamic_fluidcoef_runtime_buffers",
]
