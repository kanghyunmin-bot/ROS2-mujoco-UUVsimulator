"""Headless real-time loop for the MuJoCo runner."""

from __future__ import annotations

import time
from typing import Any, Callable


def run_headless_loop(
    *,
    stop_event: Any,
    viewer_controls: Any,
    timestep: float,
    run_step: Callable[[bool], tuple[float, float, float, float]],
) -> None:
    """Run the real-time headless loop without owning shutdown resources."""

    target_dt = float(max(timestep, 1e-6))
    next_wall = time.perf_counter()
    while not stop_event.is_set():
        is_paused = bool(viewer_controls.enable_pause and viewer_controls.paused)
        run_step(is_paused)
        next_wall += target_dt
        now_wall = time.perf_counter()
        sleep_s = next_wall - now_wall
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_wall = now_wall


__all__ = ["run_headless_loop"]
