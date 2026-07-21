"""Cadence values for the passive MuJoCo viewer loop."""

from __future__ import annotations

import os
from dataclasses import dataclass

DEFAULT_CATCHUP_WINDOW_S = 0.250
DEFAULT_SENSOR_CATCHUP_WINDOW_S = 0.250
DEFAULT_MAX_SLEEP_S = 0.001


@dataclass(frozen=True)
class ViewerLoopCadence:
    target_dt: float
    sensor_dt: float
    viewer_dt: float
    max_catchup_steps: int
    max_sensor_catchup: int
    max_step_lag_s: float
    max_sensor_lag_s: float
    max_sleep_s: float


def bounded_viewer_fps(viewer_fps: float) -> float:
    return min(max(float(viewer_fps), 5.0), 240.0)


def _env_float(name: str, default: float, *, min_value: float, max_value: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except (TypeError, ValueError):
        value = float(default)
    return max(float(min_value), min(float(max_value), value))


def _env_int(name: str, default: int, *, min_value: int, max_value: int) -> int:
    try:
        value = int(float(os.environ.get(name, default)))
    except (TypeError, ValueError):
        value = int(default)
    return max(int(min_value), min(int(max_value), value))


def physics_catchup_window_s() -> float:
    return _env_float(
        "UUV_MUJOCO_CATCHUP_WINDOW_S",
        DEFAULT_CATCHUP_WINDOW_S,
        min_value=0.020,
        max_value=2.000,
    )


def sensor_catchup_window_s() -> float:
    return _env_float(
        "UUV_MUJOCO_SENSOR_CATCHUP_WINDOW_S",
        DEFAULT_SENSOR_CATCHUP_WINDOW_S,
        min_value=0.050,
        max_value=2.000,
    )


def max_catchup_steps(target_dt: float) -> int:
    default_steps = int(round(physics_catchup_window_s() / max(float(target_dt), 1e-6)))
    return _env_int("UUV_MUJOCO_MAX_CATCHUP_STEPS", default_steps, min_value=2, max_value=2048)


def max_sensor_catchup_steps(sensor_dt: float) -> int:
    default_steps = int(round(sensor_catchup_window_s() / max(float(sensor_dt), 1e-6)))
    return _env_int("UUV_MUJOCO_MAX_SENSOR_CATCHUP", default_steps, min_value=2, max_value=512)


def max_step_lag_s(target_dt: float) -> float:
    return _env_float(
        "UUV_MUJOCO_MAX_STEP_LAG_S",
        physics_catchup_window_s(),
        min_value=max(float(target_dt), 1e-6),
        max_value=2.000,
    )


def max_sensor_lag_s(sensor_dt: float) -> float:
    return _env_float(
        "UUV_MUJOCO_MAX_SENSOR_LAG_S",
        sensor_catchup_window_s(),
        min_value=max(float(sensor_dt), 1e-6),
        max_value=2.000,
    )


def max_loop_sleep_s(target_dt: float) -> float:
    return _env_float(
        "UUV_MUJOCO_MAX_SLEEP_S",
        min(DEFAULT_MAX_SLEEP_S, max(0.0005, float(target_dt))),
        min_value=0.0,
        max_value=0.010,
    )


def build_viewer_loop_cadence(*, timestep: float, ros2_sensor_hz: float, viewer_fps: float) -> ViewerLoopCadence:
    target_dt = float(max(timestep, 1e-6))
    sensor_hz = float(max(ros2_sensor_hz, 1.0))
    sensor_dt = 1.0 / sensor_hz
    viewer_dt = 1.0 / bounded_viewer_fps(viewer_fps)
    return ViewerLoopCadence(
        target_dt=target_dt,
        sensor_dt=sensor_dt,
        viewer_dt=viewer_dt,
        max_catchup_steps=max_catchup_steps(target_dt),
        max_sensor_catchup=max_sensor_catchup_steps(sensor_dt),
        max_step_lag_s=max_step_lag_s(target_dt),
        max_sensor_lag_s=max_sensor_lag_s(sensor_dt),
        max_sleep_s=max_loop_sleep_s(target_dt),
    )


__all__ = [
    "DEFAULT_CATCHUP_WINDOW_S",
    "DEFAULT_MAX_SLEEP_S",
    "DEFAULT_SENSOR_CATCHUP_WINDOW_S",
    "ViewerLoopCadence",
    "bounded_viewer_fps",
    "build_viewer_loop_cadence",
    "max_catchup_steps",
    "max_loop_sleep_s",
    "max_sensor_catchup_steps",
    "max_sensor_lag_s",
    "max_step_lag_s",
    "physics_catchup_window_s",
    "sensor_catchup_window_s",
]
