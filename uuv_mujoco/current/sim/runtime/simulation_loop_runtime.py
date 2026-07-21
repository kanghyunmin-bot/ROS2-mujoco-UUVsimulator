"""Viewer loop orchestration for the MuJoCo runner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from sim.runtime.headless_loop_runtime import run_headless_loop
from sim.runtime.simulation_loop_viewer_runner import run_viewer_runtime_loop
from sim.runtime.viewer_loop_draw import ViewerLoopDrawMixin, normalize_vector
from sim.runtime.viewer_loop_overlay import ViewerLoopOverlayMixin


@dataclass
class ViewerRuntimeLoop(ViewerLoopDrawMixin, ViewerLoopOverlayMixin):
    """Own MuJoCo viewer frame timing, debug drawing, and overlay text."""

    mujoco: Any
    model: Any
    data: Any
    base_id: int
    act: Mapping[str, int]
    camera_ids: Mapping[str, int]
    thruster_site_ids: Mapping[str, int]
    thruster_names: Sequence[str]
    sensor_site_ids: Mapping[str, int]
    viewer_controls: Any
    stop_event: Any
    run_step: Callable[[bool, bool], tuple[float, float, float, float]]
    publish_ros_once: Callable[[], None]
    sensor_value: Callable[[str], Any]
    get_last_buoy_force: Callable[[], np.ndarray]
    get_last_buoy_point: Callable[[], np.ndarray]
    get_ros_bridge: Callable[[], Any | None]
    ros2_sensor_hz: float
    viewer_fps: float
    timestep: float
    thruster_force_max: float

    def run(self, viewer: Any) -> None:
        run_viewer_runtime_loop(self, viewer)


__all__ = ["ViewerRuntimeLoop", "normalize_vector", "run_headless_loop"]
