#!/usr/bin/env python3
"""Regression checks for low-latency simulation loop timing."""

from __future__ import annotations

import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.simulation_loop_cadence import build_viewer_loop_cadence  # noqa: E402
from sim.runtime.simulation_loop_clocks import ViewerLoopClocks  # noqa: E402
from sim.runtime.simulation_loop_viewer_runner import (  # noqa: E402
    _max_skipped_viewer_syncs,
    _sync_viewer_preserving_applied_wrenches,
    _viewer_max_catchup_steps,
)
from sim.runtime.simulation_sensor_catchup import run_sensor_catchup  # noqa: E402
from sim.runtime.simulation_step_catchup import run_simulation_catchup  # noqa: E402
from sim.runtime.headless_loop_runtime import _headless_publish_due  # noqa: E402
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


def check_headless_publish_survives_slow_physics() -> None:
    _assert(
        _headless_publish_due(now_wall=10.0, next_step_wall=9.5, next_sensor_wall=9.9),
        "headless loop must publish ROS sensors when wall-clock sensor cadence is due",
    )
    _assert(
        not _headless_publish_due(now_wall=9.8, next_step_wall=9.5, next_sensor_wall=9.9),
        "headless loop must not publish before either wall or step cadence is due",
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


def check_viewer_sync_skip_is_bounded() -> None:
    old_value = os.environ.get("UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP")
    try:
        os.environ.pop("UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP", None)
        _assert(_max_skipped_viewer_syncs() == 0, "30Hz viewer must not skip frames by default")
        os.environ["UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP"] = "1000"
        _assert(_max_skipped_viewer_syncs() == 4, "viewer sync skip override must be capped")
        os.environ["UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP"] = "-5"
        _assert(_max_skipped_viewer_syncs() == 0, "viewer sync skip override must allow no skipping")
    finally:
        if old_value is None:
            os.environ.pop("UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP", None)
        else:
            os.environ["UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP"] = old_value


def check_viewer_catchup_batch_stays_render_responsive() -> None:
    old_value = os.environ.get("UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS")
    try:
        os.environ.pop("UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS", None)
        _assert(_viewer_max_catchup_steps(125) == 4, "viewer must render between short physics batches")
        os.environ["UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS"] = "20"
        _assert(_viewer_max_catchup_steps(125) == 20, "viewer catch-up override must be honored")
        os.environ["UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS"] = "1000"
        _assert(_viewer_max_catchup_steps(125) == 125, "viewer catch-up must not exceed the global budget")
        os.environ["UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS"] = "1"
        _assert(_viewer_max_catchup_steps(125) == 2, "viewer catch-up must retain forward progress")
    finally:
        if old_value is None:
            os.environ.pop("UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS", None)
        else:
            os.environ["UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS"] = old_value


def check_state_only_viewer_sync_preserves_external_wrenches() -> None:
    class ClearingViewer:
        def __init__(self, data) -> None:
            self.data = data
            self.calls: list[bool] = []

        def sync(self, *, state_only: bool) -> None:
            self.calls.append(state_only)
            self.data.xfrc_applied[:] = 0.0
            self.data.qfrc_applied[:] = 0.0

    data = SimpleNamespace(
        xfrc_applied=np.array([[1.0, -2.0, 3.0, 0.1, 0.2, 0.3]], dtype=np.float64),
        qfrc_applied=np.array([0.4, -0.5], dtype=np.float64),
    )
    expected_xfrc = data.xfrc_applied.copy()
    expected_qfrc = data.qfrc_applied.copy()
    viewer = ClearingViewer(data)
    old_value = os.environ.get("UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC")
    os.environ["UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC"] = "1"
    try:
        _sync_viewer_preserving_applied_wrenches(SimpleNamespace(data=data), viewer)
    finally:
        if old_value is None:
            os.environ.pop("UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC", None)
        else:
            os.environ["UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC"] = old_value
    _assert(viewer.calls == [True], "viewer sync must retain state-only mode")
    _assert(np.array_equal(data.xfrc_applied, expected_xfrc), "viewer sync cleared body wrenches")
    _assert(np.array_equal(data.qfrc_applied, expected_qfrc), "viewer sync cleared generalized forces")


def main() -> int:
    check_viewer_cadence_can_catch_up_dropped_frames()
    check_step_catchup_drops_long_backlog_without_future_sleep()
    check_sensor_catchup_drops_long_backlog_without_future_sleep()
    check_headless_publish_survives_slow_physics()
    check_gui_low_latency_step_backlog_drop()
    check_viewer_frame_clock_advances_independently()
    check_viewer_sync_skip_is_bounded()
    check_viewer_catchup_batch_stays_render_responsive()
    check_state_only_viewer_sync_preserves_external_wrenches()
    print("sim_runtime_smooth_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
