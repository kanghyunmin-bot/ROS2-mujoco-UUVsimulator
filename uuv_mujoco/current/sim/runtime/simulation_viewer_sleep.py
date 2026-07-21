"""Low-latency sleep and frame cadence for the passive MuJoCo viewer."""

from __future__ import annotations

import time

from sim.runtime.simulation_loop_cadence import ViewerLoopCadence
from sim.runtime.simulation_loop_clocks import ViewerLoopClocks


def sleep_to_next_viewer_frame(*, cadence: ViewerLoopCadence, clocks: ViewerLoopClocks) -> None:
    now_wall = time.perf_counter()
    next_event_wall = min(clocks.next_step_wall, clocks.next_sensor_wall, clocks.next_viewer_wall)
    sleep_s = min(next_event_wall - now_wall, cadence.max_sleep_s)
    if sleep_s > 0.0:
        time.sleep(sleep_s)


def viewer_frame_due(*, clocks: ViewerLoopClocks, now_wall: float | None = None) -> bool:
    if now_wall is None:
        now_wall = time.perf_counter()
    return float(now_wall) + 1.0e-9 >= clocks.next_viewer_wall


def mark_viewer_frame_synced(
    *,
    cadence: ViewerLoopCadence,
    clocks: ViewerLoopClocks,
    now_wall: float | None = None,
) -> None:
    if now_wall is None:
        now_wall = time.perf_counter()
    clocks.next_viewer_wall += cadence.viewer_dt
    if float(now_wall) - clocks.next_viewer_wall > cadence.viewer_dt:
        clocks.next_viewer_wall = float(now_wall) + cadence.viewer_dt
        return
    while float(now_wall) + 1.0e-9 >= clocks.next_viewer_wall:
        clocks.next_viewer_wall += cadence.viewer_dt


__all__ = ["mark_viewer_frame_synced", "sleep_to_next_viewer_frame", "viewer_frame_due"]
