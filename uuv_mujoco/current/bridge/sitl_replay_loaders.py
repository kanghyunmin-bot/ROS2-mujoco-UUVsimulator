"""CSV loaders for full-runtime SITL sensor and VPD replay."""

from __future__ import annotations

from typing import Callable, Optional

from bridge.sitl_replay_csv_loader import load_replay_csv_records
from bridge.sitl_replay_row_parsers import native_vpd_event_from_row, sensor_frame_from_row
from bridge.sitl_replay_types import NativeVisionDeltaEvent, SensorReplayFrame


def load_sensor_replay_preview(
    path_raw: str,
    *,
    surface_pressure_pa: float,
    water_density: float,
    gravity: float,
    home_alt_m: float,
    log: Optional[Callable[[str], None]] = None,
) -> list[SensorReplayFrame]:
    """Load a sensor_replay_sitl_json.py preview for full-runtime parity."""
    return load_replay_csv_records(
        path_raw,
        missing_message="[sitl_transport] sensor replay preview missing",
        failure_message="[sitl_transport] failed to load sensor replay preview",
        row_parser=lambda row: sensor_frame_from_row(
            row,
            surface_pressure_pa=surface_pressure_pa,
            water_density=water_density,
            gravity=gravity,
            home_alt_m=home_alt_m,
        ),
        sort_key=lambda frame: frame.t_s,
        log=log,
    )


def load_native_vision_delta_events(
    path_raw: str,
    *,
    real_start_s: float,
    log: Optional[Callable[[str], None]] = None,
) -> list[NativeVisionDeltaEvent]:
    """Load recorded VISION_POSITION_DELTA events for full-runtime parity."""
    return load_replay_csv_records(
        path_raw,
        missing_message="[sitl_transport] native VPD replay CSV missing",
        failure_message="[sitl_transport] failed to load native VPD replay CSV",
        row_parser=lambda row: native_vpd_event_from_row(row, real_start_s=real_start_s),
        sort_key=lambda event: event.t_replay_s,
        log=log,
    )


__all__ = [
    "load_native_vision_delta_events",
    "load_replay_csv_records",
    "load_sensor_replay_preview",
]
