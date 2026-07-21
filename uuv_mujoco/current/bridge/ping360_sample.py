"""Ping360 sample contract."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .ping360_constants import PING360_DEG_PER_GRAD
from .ping360_effective_settings import Ping360EffectiveSettings


@dataclass
class Ping360Sample:
    sim_time_s: float
    angle_grad: int
    profile: np.ndarray
    image: np.ndarray
    ranges_m: np.ndarray
    intensities: np.ndarray
    settings: Ping360EffectiveSettings
    ping_number: int
    updated: bool

    def status_dict(self) -> dict[str, Any]:
        return {
            "sim_time_s": self.sim_time_s,
            "angle_grad": self.angle_grad,
            "angle_deg": self.angle_grad * PING360_DEG_PER_GRAD,
            "ping_number": self.ping_number,
            "settings": self.settings.as_dict(),
        }


__all__ = ["Ping360Sample"]
