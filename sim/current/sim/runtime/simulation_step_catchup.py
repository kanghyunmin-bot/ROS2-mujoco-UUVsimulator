"""Simulation-step catch-up loop for the passive viewer."""

from __future__ import annotations

import os
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
        publish_ros = clocks.next_step_wall + 1.0e-9 >= clocks.next_sensor_wall
        axis = run_step(False, publish_ros)
        if publish_ros:
            clocks.next_sensor_wall += cadence.sensor_dt
        clocks.next_step_wall += cadence.target_dt
        step_count += 1
        now_wall = time.perf_counter()
    if step_count >= cadence.max_catchup_steps and now_wall >= clocks.next_step_wall:
        _drop_excess_step_lag(cadence=cadence, clocks=clocks, now_wall=now_wall)
    return axis, now_wall


def _drop_excess_step_lag(*, cadence: ViewerLoopCadence, clocks: ViewerLoopClocks, now_wall: float) -> None:
    if _drop_excess_step_lag_to_wall_time():
        clocks.next_step_wall = float(now_wall) + float(cadence.target_dt)
        return
    earliest_next_step = float(now_wall) - float(cadence.max_step_lag_s)
    if clocks.next_step_wall < earliest_next_step:
        clocks.next_step_wall = earliest_next_step


def _drop_excess_step_lag_to_wall_time() -> bool:
    return os.environ.get("UUV_MUJOCO_DROP_EXCESS_STEP_LAG", "").strip().lower() in {"1", "true", "yes", "on"}


__all__ = ["run_simulation_catchup"]
