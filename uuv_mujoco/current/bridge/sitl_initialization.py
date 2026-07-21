"""Compatibility entry point for SITL transport sensor replay initialization."""

from __future__ import annotations

from bridge.sitl_initialization_config import (
    configure_native_vpd_start_policy,
    configure_sensor_replay_clock,
    configure_sensor_replay_reply_policy,
    configure_sensor_replay_timing,
)
from bridge.sitl_initialization_loaders import load_native_vpd_events, load_sensor_replay_frames
from bridge.sitl_initialization_logging import (
    enforce_immediate_reply_clock_contract,
    log_native_vpd_replay_if_active,
    log_sensor_replay_if_active,
)
from bridge.sitl_initialization_state import reset_sensor_replay_runtime_state


def initialize_sensor_replay_state(
    transport: object,
    *,
    surface_pressure_pa: float,
    water_density: float,
    gravity: float,
    home_alt_m: float,
) -> None:
    transport._sensor_replay_frames = load_sensor_replay_frames(
        surface_pressure_pa=float(surface_pressure_pa),
        water_density=float(water_density),
        gravity=float(gravity),
        home_alt_m=float(home_alt_m),
    )
    configure_sensor_replay_timing(transport)
    configure_sensor_replay_reply_policy(transport)
    transport._native_vpd_events = load_native_vpd_events(transport._sensor_replay_real_start_s)
    configure_native_vpd_start_policy(transport)
    configure_sensor_replay_clock(transport)
    log_native_vpd_replay_if_active(transport)
    reset_sensor_replay_runtime_state(transport)
    log_sensor_replay_if_active(transport)
    enforce_immediate_reply_clock_contract(transport)


__all__ = ["initialize_sensor_replay_state"]
