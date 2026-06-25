"""Clock state and catch-up helpers for the passive MuJoCo viewer loop."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable


AxisCommand = tuple[float, float, float, float]


@dataclass
class ViewerLoopClocks:
    next_step_wall: float
    next_sensor_wall: float
    next_viewer_wall: float


def initial_viewer_loop_clocks() -> ViewerLoopClocks:
    now = time.perf_counter()
    return ViewerLoopClocks(next_step_wall=now, next_sensor_wall=now, next_viewer_wall=now)


def run_paused_step(
    *,
    run_step: Callable[[bool, bool], AxisCommand],
    now_wall: float,
    cadence: ViewerLoopCadence,
    clocks: ViewerLoopClocks,
) -> AxisCommand:
    axis = run_step(True, False)
    clocks.next_step_wall = now_wall + cadence.target_dt
    clocks.next_sensor_wall = now_wall + cadence.sensor_dt
    return axis


__all__ = [
    "AxisCommand",
    "ViewerLoopClocks",
    "initial_viewer_loop_clocks",
    "run_paused_step",
]
