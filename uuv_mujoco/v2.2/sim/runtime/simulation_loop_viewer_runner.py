"""Passive viewer loop runner for the MuJoCo runtime."""

from __future__ import annotations

import time
from typing import Any

from sim.runtime.simulation_loop_cadence import build_viewer_loop_cadence
from sim.runtime.simulation_loop_catchup import (
    run_sensor_catchup,
    run_simulation_catchup,
    sleep_to_next_viewer_frame,
)
from sim.runtime.simulation_loop_clocks import (
    AxisCommand,
    initial_viewer_loop_clocks,
    run_paused_step,
)


def run_viewer_runtime_loop(runtime: Any, viewer: Any) -> None:
    has_set_texts = hasattr(viewer, "set_texts")
    cadence = build_viewer_loop_cadence(
        timestep=runtime.timestep,
        ros2_sensor_hz=runtime.ros2_sensor_hz,
        viewer_fps=runtime.viewer_fps,
    )
    clocks = initial_viewer_loop_clocks()
    axis: AxisCommand = (0.0, 0.0, 0.0, 0.0)

    while viewer.is_running() and not runtime.stop_event.is_set():
        axis = _advance_viewer_loop_axis(
            runtime,
            viewer=viewer,
            cadence=cadence,
            clocks=clocks,
            axis=axis,
        )
        _draw_viewer_runtime_frame(runtime, viewer=viewer, has_set_texts=has_set_texts, axis=axis)
        viewer.sync()
        sleep_to_next_viewer_frame(cadence=cadence, clocks=clocks)


def _advance_viewer_loop_axis(
    runtime: Any,
    *,
    viewer: Any,
    cadence: Any,
    clocks: Any,
    axis: AxisCommand,
) -> AxisCommand:
    now_wall = time.perf_counter()
    if runtime.viewer_controls.is_paused(viewer):
        return run_paused_step(
            run_step=runtime.run_step,
            now_wall=now_wall,
            cadence=cadence,
            clocks=clocks,
        )

    axis, now_wall = run_simulation_catchup(
        run_step=runtime.run_step,
        now_wall=now_wall,
        cadence=cadence,
        clocks=clocks,
        axis=axis,
    )
    run_sensor_catchup(
        publish_ros_once=runtime.publish_ros_once,
        now_wall=now_wall,
        cadence=cadence,
        clocks=clocks,
    )
    return axis


def _draw_viewer_runtime_frame(
    runtime: Any,
    *,
    viewer: Any,
    has_set_texts: bool,
    axis: AxisCommand,
) -> None:
    runtime.draw_scene(viewer)
    if has_set_texts:
        forward, sway, yaw, heave = axis
        runtime.update_overlay(viewer, forward=forward, sway=sway, yaw=yaw, heave=heave)


__all__ = ["run_viewer_runtime_loop"]
