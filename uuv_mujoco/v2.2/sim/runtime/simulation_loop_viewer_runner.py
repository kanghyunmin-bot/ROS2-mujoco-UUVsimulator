"""Passive viewer loop runner for the MuJoCo runtime."""

from __future__ import annotations

import os
import time
from typing import Any

from sim.runtime.simulation_loop_cadence import build_viewer_loop_cadence
from sim.runtime.simulation_loop_catchup import (
    mark_viewer_frame_synced,
    run_sensor_catchup,
    run_simulation_catchup,
    sleep_to_next_viewer_frame,
    viewer_frame_due,
)
from sim.runtime.simulation_loop_clocks import (
    AxisCommand,
    initial_viewer_loop_clocks,
    run_paused_step,
)


def run_viewer_runtime_loop(runtime: Any, viewer: Any) -> None:
    has_set_texts = (
        hasattr(viewer, "set_texts")
        and str(os.environ.get("UUV_MUJOCO_VIEWER_TEXT_OVERLAY", "0")).strip().lower() in {"1", "true", "yes", "on"}
    )
    cadence = build_viewer_loop_cadence(
        timestep=runtime.timestep,
        ros2_sensor_hz=runtime.ros2_sensor_hz,
        viewer_fps=runtime.viewer_fps,
    )
    print(
        "[runtime] viewer loop cadence: "
        f"step_dt={cadence.target_dt:.4f}s sensor_dt={cadence.sensor_dt:.4f}s "
        f"viewer_dt={cadence.viewer_dt:.4f}s catchup_steps={cadence.max_catchup_steps} "
        f"sensor_catchup={cadence.max_sensor_catchup} "
        f"max_step_lag={cadence.max_step_lag_s:.3f}s max_sensor_lag={cadence.max_sensor_lag_s:.3f}s",
        flush=True,
    )
    clocks = initial_viewer_loop_clocks()
    axis: AxisCommand = (0.0, 0.0, 0.0, 0.0)
    synced_frames = 0

    while viewer.is_running() and not runtime.stop_event.is_set():
        now_wall = time.perf_counter()
        axis, now_wall = _advance_viewer_loop_axis(
            runtime,
            viewer=viewer,
            cadence=cadence,
            clocks=clocks,
            axis=axis,
            now_wall=now_wall,
        )
        if viewer_frame_due(clocks=clocks, now_wall=now_wall):
            if synced_frames == 0 or not _viewer_frame_would_starve_runtime(
                cadence=cadence,
                clocks=clocks,
                now_wall=now_wall,
            ):
                _draw_viewer_runtime_frame(runtime, viewer=viewer, has_set_texts=has_set_texts, axis=axis)
                viewer.sync()
                synced_frames += 1
            mark_viewer_frame_synced(cadence=cadence, clocks=clocks)
        sleep_to_next_viewer_frame(cadence=cadence, clocks=clocks)


def _advance_viewer_loop_axis(
    runtime: Any,
    *,
    viewer: Any,
    cadence: Any,
    clocks: Any,
    axis: AxisCommand,
    now_wall: float,
) -> tuple[AxisCommand, float]:
    if runtime.viewer_controls.is_paused(viewer):
        axis = run_paused_step(
            run_step=runtime.run_step,
            now_wall=now_wall,
            cadence=cadence,
            clocks=clocks,
        )
        return axis, time.perf_counter()

    axis, now_wall = run_simulation_catchup(
        run_step=runtime.run_step,
        now_wall=now_wall,
        cadence=cadence,
        clocks=clocks,
        axis=axis,
    )
    now_wall = run_sensor_catchup(
        publish_ros_once=runtime.publish_ros_once,
        now_wall=now_wall,
        cadence=cadence,
        clocks=clocks,
    )
    return axis, now_wall


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


def _viewer_frame_would_starve_runtime(*, cadence: Any, clocks: Any, now_wall: float) -> bool:
    """Prioritize physics and ROS sensor publishing over native viewer frames."""

    step_lag = float(now_wall) - float(clocks.next_step_wall)
    sensor_lag = float(now_wall) - float(clocks.next_sensor_wall)
    return step_lag > float(cadence.target_dt) or sensor_lag > float(cadence.sensor_dt)


__all__ = ["run_viewer_runtime_loop"]
