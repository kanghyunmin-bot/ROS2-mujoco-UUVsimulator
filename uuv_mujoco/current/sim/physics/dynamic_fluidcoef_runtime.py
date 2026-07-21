"""Runtime updater for MuJoCo dynamic fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.dynamic_fluidcoef_runtime_config import configure_dynamic_fluidcoef_runtime
from sim.physics.dynamic_fluidcoef_runtime_update import (
    apply_dynamic_fluidcoef_target,
    apply_dynamic_fluidcoef_transient,
    compute_dynamic_fluidcoef_blend,
    dynamic_fluidcoef_update_due,
    log_dynamic_fluidcoef_update,
)
from sim.physics.dynamic_fluidcoef_types import DynamicFluidcoefSetup


class DynamicFluidcoefRuntime:
    """Runtime updater for MuJoCo dynamic fluid coefficients."""

    def __init__(
        self,
        *,
        model,
        data,
        mujoco_module,
        setup: DynamicFluidcoefSetup,
        water_current_world: np.ndarray,
        fluid_geom_names: dict[int, str],
        env_float: Callable[[str, float], float],
        env_flag: Callable[[str, bool], bool],
        to_float_array: Callable[[object], np.ndarray | None],
    ) -> None:
        self.model = model
        self.data = data
        self.mujoco = mujoco_module
        configure_dynamic_fluidcoef_runtime(
            self,
            setup=setup,
            water_current_world=water_current_world,
            fluid_geom_names=fluid_geom_names,
            env_float=env_float,
            env_flag=env_flag,
            to_float_array=to_float_array,
        )

    def update(self, _rel_lin_vel_body: np.ndarray, _ang_vel_body: np.ndarray) -> None:
        """Update each MuJoCo ellipsoid fluidcoef from transient flow onset."""
        if not self.enabled or self.active_idx.size == 0:
            return

        sim_t = float(self.data.time)
        if not dynamic_fluidcoef_update_due(self, sim_t):
            return

        idx = self.active_idx
        blend, first_loads = compute_dynamic_fluidcoef_blend(self, idx)
        transient, reset_mask = apply_dynamic_fluidcoef_transient(self, idx, blend)
        apply_dynamic_fluidcoef_target(self, idx, blend, transient, reset_mask)
        log_dynamic_fluidcoef_update(
            self,
            sim_t=sim_t,
            idx=idx,
            first_loads=first_loads,
            transient=transient,
        )
