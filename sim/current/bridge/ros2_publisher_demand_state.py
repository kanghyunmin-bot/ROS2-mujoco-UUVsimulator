"""State objects for ROS2 publisher demand caching."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DemandState:
    next_probe_t: float
    has_subscribers: bool


def demand_probe_period(default_probe_period_s: float, probe_period_s: float | None) -> float:
    if probe_period_s is None:
        return float(default_probe_period_s)
    return max(float(probe_period_s), 0.0)


__all__ = ["DemandState", "demand_probe_period"]
