"""Synthetic Blue Robotics Ping360 model for the MuJoCo runtime."""

from __future__ import annotations

import mujoco

from .ping360_sim_lifecycle import (
    initialize_ping360_simulator,
    ping360_simulator_active,
    refresh_ping360_simulator_runtime,
    update_ping360_simulator_config,
)
from .ping360_sim_status import current_ping360_angle_grad, ping360_status_dict
from .ping360_sim_update import update_ping360_simulator
from .ping360_types import (
    PING360_GRADS_PER_REV,
    Ping360Config,
    Ping360Sample,
)


class Ping360Simulator:
    """Generate Ping360-like profile scans from MuJoCo ray casts."""

    def __init__(self, model: mujoco.MjModel, config: Ping360Config) -> None:
        initialize_ping360_simulator(self, model, config)

    @property
    def active(self) -> bool:
        return ping360_simulator_active(self)

    def update_config(self, config: Ping360Config) -> None:
        update_ping360_simulator_config(self, config)

    def status_dict(self, sim_t: float | None = None) -> dict[str, object]:
        return ping360_status_dict(self, sim_t)

    def update(self, data: mujoco.MjData, sim_t: float) -> Ping360Sample | None:
        return update_ping360_simulator(self, data, sim_t)

    def _refresh_runtime_state(self) -> None:
        refresh_ping360_simulator_runtime(self)

    def _current_angle_grad(self) -> int:
        return current_ping360_angle_grad(self)

    def _advance_angle(self) -> None:
        self._runtime.sweep.advance(settings=self.settings, config=self.config)


__all__ = ["PING360_GRADS_PER_REV", "Ping360Config", "Ping360Sample", "Ping360Simulator"]
