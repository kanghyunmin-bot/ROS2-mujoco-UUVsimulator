"""Sensor replay file loaders used during SITL transport initialization."""

from __future__ import annotations

import os

from bridge.sitl_replay import load_native_vision_delta_events, load_sensor_replay_preview


def load_sensor_replay_frames(
    *,
    surface_pressure_pa: float,
    water_density: float,
    gravity: float,
    home_alt_m: float,
) -> list[object]:
    return load_sensor_replay_preview(
        os.getenv("ROS2_UUV_SITL_SENSOR_REPLAY_PREVIEW_CSV", "").strip(),
        surface_pressure_pa=float(surface_pressure_pa),
        water_density=float(water_density),
        gravity=float(gravity),
        home_alt_m=float(home_alt_m),
        log=lambda message: print(message, flush=True),
    )


def load_native_vpd_events(real_start_s: float) -> list[object]:
    return load_native_vision_delta_events(
        os.getenv("ROS2_UUV_SITL_SENSOR_REPLAY_VPD_CSV", "").strip(),
        real_start_s=real_start_s,
        log=lambda message: print(message, flush=True),
    )


__all__ = ["load_native_vpd_events", "load_sensor_replay_frames"]
