"""Headless real-time loop for the MuJoCo runner."""

from __future__ import annotations

import time
from typing import Any, Callable

from sim.runtime.simulation_loop_cadence import build_viewer_loop_cadence


def run_headless_loop(
    *,
    stop_event: Any,
    viewer_controls: Any,
    timestep: float,
    ros2_sensor_hz: float,
    run_step: Callable[..., tuple[float, float, float, float]],
) -> None:
    """Run the real-time headless loop without owning shutdown resources."""

    cadence = build_viewer_loop_cadence(timestep=timestep, ros2_sensor_hz=ros2_sensor_hz, viewer_fps=10.0)
    print(
        "[runtime] headless loop cadence: "
        f"step_dt={cadence.target_dt:.4f}s sensor_dt={cadence.sensor_dt:.4f}s "
        f"catchup_steps={cadence.max_catchup_steps} "
        f"max_step_lag={cadence.max_step_lag_s:.3f}s max_sensor_lag={cadence.max_sensor_lag_s:.3f}s",
        flush=True,
    )
    next_step_wall = time.perf_counter()
    next_sensor_wall = next_step_wall
    while not stop_event.is_set():
        now_wall = time.perf_counter()
        # Headless publishing is sampled from physics steps, so only the step
        # clock should wake the loop.  A stale sensor clock here would busy-spin.
        next_event_wall = next_step_wall
        if now_wall + 1.0e-9 < next_event_wall:
            time.sleep(min(next_event_wall - now_wall, cadence.max_sleep_s))
            continue

        is_paused = bool(viewer_controls.enable_pause and viewer_controls.paused)
        if is_paused:
            publish_ros = now_wall + 1.0e-9 >= next_sensor_wall
            run_step(True, publish_ros)
            if publish_ros:
                next_sensor_wall = _next_due_after_publish(
                    next_due=next_sensor_wall,
                    dt=cadence.sensor_dt,
                    now_wall=time.perf_counter(),
                    max_lag_s=cadence.max_sensor_lag_s,
                    drop_to_wall=True,
                )
            next_step_wall = time.perf_counter() + cadence.target_dt
            continue

        step_count = 0
        while now_wall + 1.0e-9 >= next_step_wall and step_count < cadence.max_catchup_steps:
            publish_ros = _headless_publish_due(
                now_wall=now_wall,
                next_step_wall=next_step_wall,
                next_sensor_wall=next_sensor_wall,
            )
            run_step(False, publish_ros)
            if publish_ros:
                next_sensor_wall = _next_due_after_publish(
                    next_due=next_sensor_wall,
                    dt=cadence.sensor_dt,
                    now_wall=time.perf_counter(),
                    max_lag_s=cadence.max_sensor_lag_s,
                    drop_to_wall=True,
                )
            next_step_wall += cadence.target_dt
            step_count += 1
            now_wall = time.perf_counter()
        if step_count >= cadence.max_catchup_steps and now_wall >= next_step_wall:
            next_step_wall = _drop_excess_lag(
                next_due=next_step_wall,
                now_wall=now_wall,
                max_lag_s=cadence.max_step_lag_s,
            )
        if now_wall >= next_sensor_wall:
            next_sensor_wall = _drop_excess_lag(
                next_due=next_sensor_wall,
                now_wall=now_wall,
                max_lag_s=cadence.max_sensor_lag_s,
                dt=cadence.sensor_dt,
            )


def _headless_publish_due(*, now_wall: float, next_step_wall: float, next_sensor_wall: float) -> bool:
    return (
        float(now_wall) + 1.0e-9 >= float(next_sensor_wall)
        or float(next_step_wall) + 1.0e-9 >= float(next_sensor_wall)
    )


def _next_due_after_publish(
    *,
    next_due: float,
    dt: float,
    now_wall: float,
    max_lag_s: float,
    drop_to_wall: bool = False,
) -> float:
    next_due = float(next_due) + float(dt)
    return _drop_excess_lag(
        next_due=next_due,
        now_wall=now_wall,
        max_lag_s=max_lag_s,
        dt=dt,
        drop_to_wall=drop_to_wall,
    )


def _drop_excess_lag(
    *,
    next_due: float,
    now_wall: float,
    max_lag_s: float,
    dt: float | None = None,
    drop_to_wall: bool = False,
) -> float:
    earliest_next_wall = float(now_wall) - float(max_lag_s)
    if float(next_due) < earliest_next_wall:
        if drop_to_wall:
            return float(now_wall) + float(dt if dt is not None else 0.0)
        return earliest_next_wall
    return float(next_due)


__all__ = ["run_headless_loop"]
