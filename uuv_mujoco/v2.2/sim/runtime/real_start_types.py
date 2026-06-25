"""Shared types for real-start runtime contracts."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


EnvFloat = Callable[[str, float], float]
EnvFlag = Callable[[str, bool], bool]


@dataclass(frozen=True)
class RealStartTargets:
    target_depth: float
    depth_contract: str
    target_rpy: tuple[float, float, float]
    target_x: float
    target_y: float
    target_pressure_pa: float
    target_v: np.ndarray
    target_w: np.ndarray
    source_t_s: float
    pressure_tol_pa: float
    xy_tol_m: float


@dataclass(frozen=True)
class RealStartMeasurements:
    depth_now: float
    depth_error: float
    base_xy_now: np.ndarray
    xy_error: float
    pressure_now_pa: float
    pressure_error_pa: float
    attitude_error: float
    velocity_error: float
    angular_velocity_error: float


__all__ = ["EnvFlag", "EnvFloat", "RealStartMeasurements", "RealStartTargets"]
