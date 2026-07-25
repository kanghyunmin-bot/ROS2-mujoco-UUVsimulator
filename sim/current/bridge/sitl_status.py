"""Compatibility exports for SITL transport telemetry status builders."""

from __future__ import annotations

from .sitl_status_age import wall_age_s
from .sitl_status_mavlink import build_mavlink_telemetry_status
from .sitl_status_sensor_replay import build_sensor_replay_status


__all__ = ["build_mavlink_telemetry_status", "build_sensor_replay_status", "wall_age_s"]
