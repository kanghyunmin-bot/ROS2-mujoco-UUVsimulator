"""Shared types for the MuJoCo <-> ArduSub SITL bridge."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VerticalEstimate:
    """Single-source vertical state exported to ArduSub SITL."""

    depth_m: float
    pressure_pa: float | None
    pos_ned: np.ndarray
    vel_ned: np.ndarray
    alt_m: float
    extnav_pos_ned: np.ndarray | None = None
