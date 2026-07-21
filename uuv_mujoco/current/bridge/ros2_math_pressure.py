"""Pressure conversion helpers for the SITL sensor contract."""

from __future__ import annotations


def pressure_abs_from_depth_m(depth_m: float, surface_pressure_pa: float, rho: float, gravity: float) -> float:
    depth = float(max(0.0, depth_m))
    return float(surface_pressure_pa + rho * gravity * depth)
