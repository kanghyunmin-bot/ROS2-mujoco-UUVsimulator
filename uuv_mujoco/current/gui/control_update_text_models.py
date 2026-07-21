"""Text payload types for GUI telemetry refresh."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlUpdateTexts:
    status: str
    mode: str
    vehicle_summary: str
    battery: str
    pose: str
    velocity: str
    imu: str
    motion_summary: str
    autopilot: str
    depth_target: str
    depth_source: str
    age: str
    ping360_summary: str
    control_summary: str
    control: str
    rc_override: str


@dataclass(frozen=True)
class TelemetryTexts:
    status: str
    mode: str
    vehicle_summary: str
    battery: str
    pose: str
    velocity: str
    imu: str
    motion_summary: str
    autopilot: str
    depth_target: str
    depth_source: str
    age: str
    ping360_summary: str


@dataclass(frozen=True)
class PilotControlTexts:
    control_summary: str
    control: str
    rc_override: str


__all__ = ["ControlUpdateTexts", "PilotControlTexts", "TelemetryTexts"]
