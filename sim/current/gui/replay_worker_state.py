"""Mutable state for the GUI RC replay worker loop."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ReplayWorkerState:
    local_duration_s: float
    idx: int
    start_wall: float
    paused_since: float | None = None
    last_status_wall: float = 0.0
    last_position_wall: float = 0.0
    stopped: bool = False


__all__ = ["ReplayWorkerState"]
