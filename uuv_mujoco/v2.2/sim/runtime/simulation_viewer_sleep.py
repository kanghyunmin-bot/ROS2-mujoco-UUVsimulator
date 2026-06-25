"""Viewer frame sleep cadence for the passive MuJoCo viewer."""

from __future__ import annotations

import time

from sim.runtime.simulation_loop_cadence import ViewerLoopCadence
from sim.runtime.simulation_loop_clocks import ViewerLoopClocks


def sleep_to_next_viewer_frame(*, cadence: ViewerLoopCadence, clocks: ViewerLoopClocks) -> None:
    clocks.next_viewer_wall += cadence.viewer_dt
    now_wall = time.perf_counter()
    sleep_s = clocks.next_viewer_wall - now_wall
    if sleep_s > 0.0:
        time.sleep(sleep_s)
    else:
        clocks.next_viewer_wall = now_wall


__all__ = ["sleep_to_next_viewer_frame"]
