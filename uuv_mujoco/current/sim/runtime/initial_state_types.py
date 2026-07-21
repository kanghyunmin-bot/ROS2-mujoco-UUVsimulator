"""Typed outputs for initial MuJoCo runtime state setup."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class InitialRuntimeState:
    initial_depth_hold: dict
    initial_depth_hold_auto_release: bool
    real_start_required: bool
    real_start_depth_tol_m: float
    real_start_attitude_tol_rad: float
    real_start_velocity_tol_mps: float


@dataclass(frozen=True)
class InitialRealStartPolicy:
    auto_release: bool
    required: bool
    depth_tol_m: float
    attitude_tol_rad: float
    velocity_tol_mps: float


__all__ = ["InitialRealStartPolicy", "InitialRuntimeState"]
