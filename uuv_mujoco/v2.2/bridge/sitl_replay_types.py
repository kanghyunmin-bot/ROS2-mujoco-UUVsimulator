"""Typed records for full-runtime SITL parity replay."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SensorReplayFrame:
    """Controller-parity replay sample already converted to SITL JSON fields."""

    t_s: float
    gyro_frd: np.ndarray
    accel_frd: np.ndarray
    quat_ned_frd: np.ndarray
    depth_m: float
    pressure_pa: float | None
    pos_ned: np.ndarray
    vel_ned: np.ndarray
    alt_m: float
    extnav_pos_ned: np.ndarray | None


@dataclass(frozen=True)
class NativeVisionDeltaEvent:
    """Recorded MAVLink VISION_POSITION_DELTA event for controller parity."""

    t_real_s: float
    t_replay_s: float
    time_usec: int
    time_delta_usec: int
    angle_delta: np.ndarray
    position_delta: np.ndarray
    confidence: float


__all__ = ["NativeVisionDeltaEvent", "SensorReplayFrame"]
