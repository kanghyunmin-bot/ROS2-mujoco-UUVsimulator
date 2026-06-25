"""Simulation-step catch-up loop for the passive viewer."""

from __future__ import annotations

import time
from typing import Callable

from sim.runtime.simulation_loop_cadence import ViewerLoopCadence
from sim.runtime.simulation_loop_clocks import AxisCommand, ViewerLoopClocks


def run_simulation_catchup(
    *,
    run_step: Callable[[bool, bool], AxisCommand],
    now_wall: float,
    cadence: ViewerLoopCadence,
    clocks: ViewerLoopClocks,
    axis: AxisCommand,
) -> tuple[AxisCommand, float]:
    step_count = 0
    while now_wall >= clocks.next_step_wall and step_count < cadence.max_catchup_steps:
        axis = run_step(False, False)
        clocks.next_step_wall += cadence.target_dt
        step_count += 1
        now_wall = time.perf_counter()
    if step_count >= cadence.max_catchup_steps and now_wall >= clocks.next_step_wall:
        clocks.next_step_wall = now_wall + cadence.target_dt
    return axis, now_wall


__all__ = ["run_simulation_catchup"]
