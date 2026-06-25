"""Ping360 sweep angle state."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ping360_types import PING360_GRADS_PER_REV, Ping360Config, Ping360EffectiveSettings


@dataclass
class Ping360SweepState:
    full_angle_grad: int = 0
    sector_rel_grad: int = 0
    sector_direction: int = 1

    def current_angle_grad(self, settings: Ping360EffectiveSettings) -> int:
        if settings.sector_size_grad >= PING360_GRADS_PER_REV:
            return (settings.start_angle_grad + self.full_angle_grad) % PING360_GRADS_PER_REV
        rel = int(np.clip(self.sector_rel_grad, 0, max(0, settings.sector_size_grad - 1)))
        return (settings.start_angle_grad + rel) % PING360_GRADS_PER_REV

    def advance(self, *, settings: Ping360EffectiveSettings, config: Ping360Config) -> None:
        steps = int(settings.num_steps)
        if settings.sector_size_grad >= PING360_GRADS_PER_REV:
            self.full_angle_grad = (self.full_angle_grad + steps) % PING360_GRADS_PER_REV
            return
        max_rel = max(0, settings.sector_size_grad - 1)
        if not config.sector_bounce:
            self.sector_rel_grad = (self.sector_rel_grad + steps) % max(1, settings.sector_size_grad)
            return
        next_rel = self.sector_rel_grad + self.sector_direction * steps
        if next_rel < 0 or next_rel > max_rel:
            self.sector_direction *= -1
            next_rel = self.sector_rel_grad + self.sector_direction * steps
        self.sector_rel_grad = int(np.clip(next_rel, 0, max_rel))


__all__ = ["Ping360SweepState"]
