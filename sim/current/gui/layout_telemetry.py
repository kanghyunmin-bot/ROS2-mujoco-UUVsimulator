"""Telemetry-side Tk layout builders."""

from __future__ import annotations

from .layout_event_log import build_event_log
from .layout_vehicle_summary import build_vehicle_summary
from .layout_vehicle_visuals import build_vehicle_visuals


def build_telemetry_panel(owner, left) -> None:
    build_vehicle_summary(owner, left)
    build_vehicle_visuals(owner, left)
    build_event_log(owner, left)
