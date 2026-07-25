"""Numeric settings extraction for Ping360 GUI status callbacks."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Ping360StatusSettings:
    effective_range: float
    requested_range: float
    resolution_cm: float
    angular_resolution: float
    scan_period: float
    angle_deg: float
    num_steps: int
    start_grad: int
    stop_grad: int
    flags: object


def extract_ping360_status_settings(payload: dict, settings: dict) -> Ping360StatusSettings:
    try:
        return Ping360StatusSettings(
            effective_range=float(settings.get("effective_range_m", math.nan)),
            requested_range=float(settings.get("requested_range_m", math.nan)),
            resolution_cm=float(settings.get("range_resolution_m", math.nan)) * 100.0,
            angular_resolution=float(settings.get("angular_resolution_deg", math.nan)),
            scan_period=float(settings.get("scan_period_s", math.nan)),
            angle_deg=float(payload.get("angle_deg", math.nan)),
            num_steps=int(settings.get("num_steps", 0)),
            start_grad=int(settings.get("start_angle_grad", 0)),
            stop_grad=int(settings.get("stop_angle_grad", 399)),
            flags=settings.get("quality_flags", []),
        )
    except (TypeError, ValueError):
        return Ping360StatusSettings(
            effective_range=math.nan,
            requested_range=math.nan,
            resolution_cm=math.nan,
            angular_resolution=math.nan,
            scan_period=math.nan,
            angle_deg=math.nan,
            num_steps=0,
            start_grad=0,
            stop_grad=399,
            flags=[],
        )


__all__ = ["Ping360StatusSettings", "extract_ping360_status_settings"]
