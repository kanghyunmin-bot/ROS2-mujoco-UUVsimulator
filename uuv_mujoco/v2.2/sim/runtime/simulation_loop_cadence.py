"""Cadence values for the passive MuJoCo viewer loop."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ViewerLoopCadence:
    target_dt: float
    sensor_dt: float
    viewer_dt: float
    max_catchup_steps: int
    max_sensor_catchup: int


def bounded_viewer_fps(viewer_fps: float) -> float:
    return min(max(float(viewer_fps), 10.0), 240.0)


def build_viewer_loop_cadence(*, timestep: float, ros2_sensor_hz: float, viewer_fps: float) -> ViewerLoopCadence:
    target_dt = float(max(timestep, 1e-6))
    sensor_hz = float(max(ros2_sensor_hz, 1.0))
    sensor_dt = 1.0 / sensor_hz
    viewer_dt = 1.0 / bounded_viewer_fps(viewer_fps)
    return ViewerLoopCadence(
        target_dt=target_dt,
        sensor_dt=sensor_dt,
        viewer_dt=viewer_dt,
        max_catchup_steps=max(4, int(round(0.10 / target_dt))),
        max_sensor_catchup=max(2, int(round(0.10 / sensor_dt))),
    )


__all__ = ["ViewerLoopCadence", "bounded_viewer_fps", "build_viewer_loop_cadence"]
