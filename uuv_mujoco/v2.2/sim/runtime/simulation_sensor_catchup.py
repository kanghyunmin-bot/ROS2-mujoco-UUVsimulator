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
        clocks.next_sensor_wall = now_wall + cadence.sensor_dt
    return now_wall


__all__ = ["run_sensor_catchup"]
