"""Shared constants for axis RC override checks."""

from __future__ import annotations

from dataclasses import dataclass


RC_NEUTRAL = 1500
RC_SPAN = 300
AXIS_TO_CHANNEL = {
    "pitch": 0,
    "roll": 1,
    "heave": 2,
    "yaw": 3,
    "forward": 4,
    "lateral": 5,
}
AXIS_ORDER = ("roll", "pitch", "yaw", "heave", "forward", "lateral")
EXPECTED_AXIS_METRIC = {
    "roll": "gyro_x",
    "pitch": "gyro_y",
    "yaw": "gyro_z",
    "heave": "dvl_vz",
    "forward": "dvl_vx",
    "lateral": "dvl_vy",
}
MIN_EXPECTED_PEAK = {
    "roll": 0.003,
    "pitch": 0.003,
    "yaw": 0.03,
    "heave": 0.01,
    "forward": 0.02,
    "lateral": 0.02,
}


@dataclass
class Phase:
    name: str
    axis: str
    command: float
    start: float
    end: float


__all__ = [
    "AXIS_ORDER",
    "AXIS_TO_CHANNEL",
    "EXPECTED_AXIS_METRIC",
    "MIN_EXPECTED_PEAK",
    "Phase",
    "RC_NEUTRAL",
    "RC_SPAN",
]
