"""Passive viewer loop runner for the MuJoCo runtime."""

from __future__ import annotations

import os
import time
from dataclasses import replace
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
    cadence = replace(
        cadence,
        max_catchup_steps=_viewer_max_catchup_steps(cadence.max_catchup_steps),
    )
    print(
        "[runtime] viewer loop cadence: "
        f"step_dt={cadence.target_dt:.4f}s wall_step_dt={cadence.wall_step_dt:.4f}s "
        f"speed={cadence.speed_factor:.2f}x sensor_dt={cadence.sensor_dt:.4f}s "
        f"viewer_dt={cadence.viewer_dt:.4f}s catchup_steps={cadence.max_catchup_steps} "
        f"sensor_catchup={cadence.max_sensor_catchup} "
        f"max_step_lag={cadence.max_step_lag_s:.3f}s max_sensor_lag={cadence.max_sensor_lag_s:.3f}s",
        flush=True,
    )
    clocks = initial_viewer_loop_clocks()
    axis: AxisCommand = (0.0, 0.0, 0.0, 0.0)
    synced_frames = 0
    skipped_syncs = 0
    stats_started_wall = time.perf_counter()
    stats_started_sim = float(runtime.data.time)
    stats_synced_frames = 0
    stats_skipped_frames = 0
    stats_timing = {"advance_s": 0.0, "render_s": 0.0, "loops": 0}
    max_skipped_syncs = _max_skipped_viewer_syncs()

    while viewer.is_running() and not runtime.stop_event.is_set():
        now_wall = time.perf_counter()
        advance_started = now_wall
        axis, now_wall = _advance_viewer_loop_axis(
            runtime,
            viewer=viewer,
            cadence=cadence,
            clocks=clocks,
            axis=axis,
            now_wall=now_wall,
        )
        stats_timing["advance_s"] += max(0.0, now_wall - advance_started)
        stats_timing["loops"] += 1
        if viewer_frame_due(clocks=clocks, now_wall=now_wall):
            should_skip_sync = synced_frames > 0 and _viewer_frame_would_starve_runtime(
                cadence=cadence,
                clocks=clocks,
                now_wall=now_wall,
            )
            if should_skip_sync and skipped_syncs < max_skipped_syncs:
                skipped_syncs += 1
                stats_skipped_frames += 1
            else:
                render_started = time.perf_counter()
                _draw_viewer_runtime_frame(runtime, viewer=viewer, has_set_texts=has_set_texts, axis=axis)
                _sync_viewer_preserving_applied_wrenches(runtime, viewer)
                stats_timing["render_s"] += max(0.0, time.perf_counter() - render_started)
                synced_frames += 1
                stats_synced_frames += 1
                skipped_syncs = 0
                now_wall = time.perf_counter()
            mark_viewer_frame_synced(cadence=cadence, clocks=clocks, now_wall=now_wall)
        stats_started_wall, stats_started_sim, stats_synced_frames, stats_skipped_frames = _report_viewer_stats(
            started_wall=stats_started_wall,
            started_sim=stats_started_sim,
            sim_time=float(runtime.data.time),
            synced_frames=stats_synced_frames,
            skipped_frames=stats_skipped_frames,
            now_wall=now_wall,
            timing=stats_timing,
        )
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
    """Skip a frame only for material backlog, not normal scheduling jitter."""

    step_lag = float(now_wall) - float(clocks.next_step_wall)
    sensor_lag = float(now_wall) - float(clocks.next_sensor_wall)
    step_budget = max(3.0 * float(cadence.wall_step_dt), 0.75 * float(cadence.viewer_dt))
    sensor_budget = max(float(cadence.sensor_dt), 0.75 * float(cadence.viewer_dt))
    return step_lag > step_budget or sensor_lag > sensor_budget


def _max_skipped_viewer_syncs() -> int:
    try:
        value = int(float(os.environ.get("UUV_MUJOCO_VIEWER_MAX_SYNC_SKIP", "0")))
    except (TypeError, ValueError):
        value = 0
    return max(0, min(4, value))


def _viewer_max_catchup_steps(base_steps: int) -> int:
    try:
        value = int(float(os.environ.get("UUV_MUJOCO_VIEWER_MAX_CATCHUP_STEPS", "4")))
    except (TypeError, ValueError):
        value = 4
    return max(2, min(int(base_steps), value, 256))


def _viewer_state_only_sync() -> bool:
    return os.environ.get("UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC", "1").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _sync_viewer_preserving_applied_wrenches(runtime: Any, viewer: Any) -> None:
    """Keep simulator-owned external forces across state-only viewer syncs."""

    state_only = _viewer_state_only_sync()
    if not state_only:
        viewer.sync(state_only=False)
        return

    data = runtime.data
    xfrc_applied = data.xfrc_applied.copy()
    qfrc_applied = data.qfrc_applied.copy() if hasattr(data, "qfrc_applied") else None
    viewer.sync(state_only=True)
    data.xfrc_applied[:] = xfrc_applied
    if qfrc_applied is not None:
        data.qfrc_applied[:] = qfrc_applied


def _report_viewer_stats(
    *,
    started_wall: float,
    started_sim: float,
    sim_time: float,
    synced_frames: int,
    skipped_frames: int,
    now_wall: float,
    timing: dict[str, float],
) -> tuple[float, float, int, int]:
    if str(os.environ.get("UUV_MUJOCO_VIEWER_STATS", "1")).strip().lower() not in {"1", "true", "yes", "on"}:
        return started_wall, started_sim, synced_frames, skipped_frames
    elapsed = float(now_wall) - float(started_wall)
    if elapsed < 5.0:
        return started_wall, started_sim, synced_frames, skipped_frames
    print(
        "[runtime] viewer stats: "
        f"sync_fps={float(synced_frames) / max(elapsed, 1.0e-9):.1f} "
        f"rtf={max(0.0, float(sim_time) - float(started_sim)) / max(elapsed, 1.0e-9):.2f} "
        f"synced={synced_frames} skipped={skipped_frames} "
        f"advance={100.0 * float(timing.get('advance_s', 0.0)) / max(elapsed, 1.0e-9):.0f}% "
        f"render={100.0 * float(timing.get('render_s', 0.0)) / max(elapsed, 1.0e-9):.0f}% "
        f"window={elapsed:.1f}s",
        flush=True,
    )
    timing["advance_s"] = 0.0
    timing["render_s"] = 0.0
    timing["loops"] = 0
    return float(now_wall), float(sim_time), 0, 0


__all__ = [
    "run_viewer_runtime_loop",
    "_max_skipped_viewer_syncs",
    "_sync_viewer_preserving_applied_wrenches",
    "_viewer_max_catchup_steps",
]
