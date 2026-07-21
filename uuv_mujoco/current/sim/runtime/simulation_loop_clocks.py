"""Clock state and catch-up helpers for the passive MuJoCo viewer loop."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from sim.runtime.simulation_loop_cadence import ViewerLoopCadence


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
    publish_ros = float(now_wall) + 1.0e-9 >= float(clocks.next_sensor_wall)
    axis = run_step(True, publish_ros)
    clocks.next_step_wall = now_wall + cadence.target_dt
    if publish_ros:
        # Advance from the scheduled deadline so pause/unpause does not drift
        # the sensor clock. Bound catch-up to one publish per viewer iteration.
        overdue_s = max(0.0, float(now_wall) - float(clocks.next_sensor_wall))
        periods = int(overdue_s / cadence.sensor_dt) + 1
        clocks.next_sensor_wall += periods * cadence.sensor_dt
    return axis


__all__ = [
    "AxisCommand",
    "ViewerLoopClocks",
    "initial_viewer_loop_clocks",
    "run_paused_step",
]
