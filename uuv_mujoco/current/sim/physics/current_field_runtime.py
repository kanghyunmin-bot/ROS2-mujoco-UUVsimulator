"""Runtime adapter between a current field and MuJoCo's single-vehicle flow."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from sim.physics.current_field import DeterministicCurrentField


class CurrentFieldRuntime:
    """Sample one field at the UUV and update the shared current in place."""

    def __init__(
        self,
        *,
        model,
        field: DeterministicCurrentField,
        water_current_world: np.ndarray,
        use_custom_hydrodynamics: bool,
        log: Callable[[str], None],
    ) -> None:
        self.model = model
        self.field = field
        self.water_current_world = water_current_world
        self.use_custom_hydrodynamics = bool(use_custom_hydrodynamics)
        self._last_logged_second = -1
        self._log = log

    def update(self, position_world_m: np.ndarray, time_s: float) -> np.ndarray:
        """Sample the field and return the current shared velocity [m/s]."""

        sample = self.field.velocity_world(position_world_m, time_s)
        self.water_current_world[:] = sample
        if not self.use_custom_hydrodynamics:
            self.model.opt.wind[:] = sample
        return self.water_current_world


__all__ = ["CurrentFieldRuntime"]
