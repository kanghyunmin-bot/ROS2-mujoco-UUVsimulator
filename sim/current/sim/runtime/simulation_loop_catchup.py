"""Compatibility facade for passive MuJoCo viewer catch-up helpers."""

from __future__ import annotations

from sim.runtime.simulation_sensor_catchup import run_sensor_catchup
from sim.runtime.simulation_step_catchup import run_simulation_catchup
from sim.runtime.simulation_viewer_sleep import (
    mark_viewer_frame_synced,
    sleep_to_next_viewer_frame,
    viewer_frame_due,
)


__all__ = [
    "run_sensor_catchup",
    "run_simulation_catchup",
    "mark_viewer_frame_synced",
    "sleep_to_next_viewer_frame",
    "viewer_frame_due",
]
