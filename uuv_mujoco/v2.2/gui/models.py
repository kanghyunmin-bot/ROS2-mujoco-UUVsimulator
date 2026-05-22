"""Data models used by the MuJoCo UUV GUI."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque

from .config import RC_FEEDBACK_CHANNEL_COUNT, TELEMETRY_EVENT_LIMIT


@dataclass
class TelemetrySnapshot:
    connected: bool = False
    armed: bool = False
    guided: bool = False
    manual_input: bool = False
    mode: str = "UNKNOWN"
    mode_id: int = -1
    vehicle_mode: str = ""
    autopilot_name: str = ""
    system_status: int = 0

    battery_voltage: float = math.nan
    battery_current: float = math.nan
    battery_percent: float = math.nan

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    ang_vel_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    lin_acc_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)

    position_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    velocity_xyz: tuple[float, float, float] = (math.nan, math.nan, math.nan)
    velocity_source: str = "unavailable"

    depth_m: float = math.nan
    depth_source: str = "unavailable"
    pressure_pa: float = math.nan

    rc_in: list[int] = field(default_factory=lambda: [0] * RC_FEEDBACK_CHANNEL_COUNT)
    rc_out: list[int] = field(default_factory=lambda: [0] * RC_FEEDBACK_CHANNEL_COUNT)
    rc_in_source: str = "unavailable"
    rc_out_source: str = "unavailable"
    rc_feedback_source: str = "unavailable"
    events: Deque[str] = field(default_factory=lambda: deque(maxlen=TELEMETRY_EVENT_LIMIT))
    ping360_summary: str = "ping360: no status"
    ping360_enabled: bool | None = None
    ping360_active: bool | None = None
    ping360_age_s: float = math.inf

    state_age_s: float = math.inf
    imu_age_s: float = math.inf
    pose_age_s: float = math.inf
    depth_age_s: float = math.inf
    rc_age_s: float = math.inf
    rc_in_age_s: float = math.inf
    rc_out_age_s: float = math.inf


@dataclass(frozen=True)
class ControlCommands:
    velocity_forward: float
    velocity_lateral: float
    velocity_heave: float
    velocity_yaw: float
    rc_forward: float
    rc_lateral: float
    rc_heave: float
    rc_yaw: float


@dataclass(frozen=True)
class RcReplaySample:
    time_s: float
    channels: tuple[int, ...]
