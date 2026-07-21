"""Deterministic fixed-rate scheduling helpers for MuJoCo sim time."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class FixedRateSimScheduler:
    """Gate work at a fixed rate using simulation time, not wall-clock time."""

    dt: float
    next_time: float = -1.0
    last_update_time: float = math.nan
    epsilon: float = 1.0e-9

    def due(self, sim_time: float, fallback_dt: float) -> tuple[bool, float]:
        sim_time = float(sim_time)
        fallback_dt = float(fallback_dt)
        if self.next_time < 0.0:
            self.next_time = sim_time
        if sim_time + self.epsilon < self.next_time:
            return False, 0.0
        while sim_time + self.epsilon >= self.next_time:
            self.next_time += self.dt
        if math.isfinite(self.last_update_time):
            update_dt = max(sim_time - self.last_update_time, fallback_dt)
        else:
            update_dt = fallback_dt
        self.last_update_time = sim_time
        return True, update_dt
