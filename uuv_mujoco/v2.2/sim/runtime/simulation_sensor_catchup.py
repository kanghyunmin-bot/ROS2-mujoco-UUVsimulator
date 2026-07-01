"""ROS sensor publish catch-up loop for the passive viewer."""

from __future__ import annotations

import time
from typing import Callable

from sim.runtime.simulation_loop_cadence import ViewerLoopCadence
from sim.runtime.simulation_loop_clocks import ViewerLoopClocks


def run_sensor_catchup(
    *,
    publish_ros_once: Callable[[], None],
    now_wall: float,
    cadence: ViewerLoopCadence,
    clocks: ViewerLoopClocks,
) -> float:
    sensor_count = 0
    while now_wall >= clocks.next_sensor_wall and sensor_count < cadence.max_sensor_catchup:
        publish_ros_once()
        clocks.next_sensor_wall += cadence.sensor_dt
        sensor_count += 1
        now_wall = time.perf_counter()
    if sensor_count >= cadence.max_sensor_catchup and now_wall >= clocks.next_sensor_wall:
        _drop_excess_sensor_lag(cadence=cadence, clocks=clocks, now_wall=now_wall)
    return now_wall


def _drop_excess_sensor_lag(*, cadence: ViewerLoopCadence, clocks: ViewerLoopClocks, now_wall: float) -> None:
    earliest_next_sensor = float(now_wall) - float(cadence.max_sensor_lag_s)
    if clocks.next_sensor_wall < earliest_next_sensor:
        clocks.next_sensor_wall = earliest_next_sensor


__all__ = ["run_sensor_catchup"]
