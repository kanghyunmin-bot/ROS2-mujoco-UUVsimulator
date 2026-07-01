#!/usr/bin/env python3
"""Regression checks for low-latency simulation loop timing."""

from __future__ import annotations

import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.simulation_loop_cadence import build_viewer_loop_cadence  # noqa: E402
from sim.runtime.simulation_loop_clocks import ViewerLoopClocks  # noqa: E402
from sim.runtime.simulation_sensor_catchup import run_sensor_catchup  # noqa: E402
from sim.runtime.simulation_step_catchup import run_simulation_catchup  # noqa: E402
from sim.runtime.simulation_viewer_sleep import (  # noqa: E402
    mark_viewer_frame_synced,
    viewer_frame_due,
)


def _assert(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)


def check_viewer_cadence_can_catch_up_dropped_frames() -> None:
    cadence = build_viewer_loop_cadence(timestep=0.002, ros2_sensor_hz=50.0, viewer_fps=60.0)
    _assert(cadence.max_catchup_steps >= 100, "sim catch-up window must cover at least 200 ms")
    _assert(cadence.max_step_lag_s >= 0.200, "sim lag cap must tolerate more than a few dropped frames")
    _assert(cadence.max_sleep_s <= 0.002, "viewer loop sleep must stay responsive to sim/input events")


def check_step_catchup_drops_long_backlog_without_future_sleep() -> None:
    cadence = build_viewer_loop_cadence(timestep=0.002, ros2_sensor_hz=50.0, viewer_fps=60.0)
    clocks = ViewerLoopClocks(next_step_wall=0.0, next_sensor_wall=0.0, next_viewer_wall=0.0)
    steps = 0
    step_publishes = 0

    def run_step(_is_paused: bool, publish_ros: bool) -> tuple[float, float, float, float]:
        nonlocal steps
        nonlocal step_publishes
        steps += 1
        if publish_ros:
            step_publishes += 1
        return (1.0, 0.0, 0.0, 0.0)

    axis, now_wall = run_simulation_catchup(
        run_step=run_step,
        now_wall=1.0,
        cadence=cadence,
        clocks=clocks,
        axis=(0.0, 0.0, 0.0, 0.0),
    )
    _assert(steps == cadence.max_catchup_steps, "sim catch-up must be bounded per loop")
    _assert(step_publishes > 0, "viewer catch-up steps must keep ROS sensor publishing alive")
    _assert(axis == (1.0, 0.0, 0.0, 0.0), "latest axis command must be returned")
    _assert(clocks.next_step_wall <= now_wall, "sim backlog handling must not schedule a future stall")
    _assert(
        clocks.next_step_wall >= now_wall - cadence.max_step_lag_s - 1.0e-9,
        "sim backlog handling must retain only bounded lag",
    )


def check_sensor_catchup_drops_long_backlog_without_future_sleep() -> None:
    cadence = build_viewer_loop_cadence(timestep=0.002, ros2_sensor_hz=50.0, viewer_fps=60.0)
    clocks = ViewerLoopClocks(next_step_wall=0.0, next_sensor_wall=0.0, next_viewer_wall=0.0)
    publishes = 0

    def publish_ros_once() -> None:
        nonlocal publishes
        publishes += 1

    now_wall = run_sensor_catchup(
        publish_ros_once=publish_ros_once,
        now_wall=1.0,
        cadence=cadence,
        clocks=clocks,
    )
    _assert(publishes == cadence.max_sensor_catchup, "sensor catch-up must be bounded per loop")
    _assert(clocks.next_sensor_wall <= now_wall, "sensor backlog handling must not schedule a future stall")
    _assert(
        clocks.next_sensor_wall >= now_wall - cadence.max_sensor_lag_s - 1.0e-9,
        "sensor backlog handling must retain only bounded lag",
    )


def check_gui_low_latency_step_backlog_drop() -> None:
    cadence = build_viewer_loop_cadence(timestep=0.008, ros2_sensor_hz=25.0, viewer_fps=24.0)
    clocks = ViewerLoopClocks(next_step_wall=0.0, next_sensor_wall=0.0, next_viewer_wall=0.0)
    old_value = os.environ.get("UUV_MUJOCO_DROP_EXCESS_STEP_LAG")
    os.environ["UUV_MUJOCO_DROP_EXCESS_STEP_LAG"] = "1"
    try:
        run_simulation_catchup(
            run_step=lambda _is_paused, _publish_ros: (0.0, 0.0, 0.0, 0.0),
            now_wall=1.0,
            cadence=cadence,
            clocks=clocks,
            axis=(0.0, 0.0, 0.0, 0.0),
        )
    finally:
        if old_value is None:
            os.environ.pop("UUV_MUJOCO_DROP_EXCESS_STEP_LAG", None)
        else:
            os.environ["UUV_MUJOCO_DROP_EXCESS_STEP_LAG"] = old_value
    _assert(clocks.next_step_wall > 1.0, "low-latency GUI mode must drop stale physics backlog to wall time")


def check_viewer_frame_clock_advances_independently() -> None:
    cadence = build_viewer_loop_cadence(timestep=0.002, ros2_sensor_hz=50.0, viewer_fps=60.0)
    clocks = ViewerLoopClocks(next_step_wall=0.0, next_sensor_wall=0.0, next_viewer_wall=1.0)
    _assert(not viewer_frame_due(clocks=clocks, now_wall=0.999), "viewer frame must wait for its own cadence")
    _assert(viewer_frame_due(clocks=clocks, now_wall=1.0), "viewer frame must draw when due")
    mark_viewer_frame_synced(cadence=cadence, clocks=clocks, now_wall=1.0)
    _assert(clocks.next_viewer_wall > 1.0, "viewer frame sync must advance only viewer cadence")


def main() -> int:
    check_viewer_cadence_can_catch_up_dropped_frames()
    check_step_catchup_drops_long_backlog_without_future_sleep()
    check_sensor_catchup_drops_long_backlog_without_future_sleep()
    check_gui_low_latency_step_backlog_drop()
    check_viewer_frame_clock_advances_independently()
    print("sim_runtime_smooth_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
