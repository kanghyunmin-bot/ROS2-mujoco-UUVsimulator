"""Small math and formatting helpers for GUI telemetry and controls."""

from __future__ import annotations

import math

from .gui_axis_normalization import clamp, clamp_axis, normalize_axes
from .replay_format import format_replay_time
from .runtime import StatusText


def format_age(age_s: float) -> str:
    if not math.isfinite(age_s):
        return "n/a"
    if age_s < 1.0:
        return f"{age_s * 1000.0:.0f} ms"
    return f"{age_s:.1f} s"


def severity_name(level: int) -> str:
    names = {
        StatusText.EMERGENCY: "EMERGENCY",
        StatusText.ALERT: "ALERT",
        StatusText.CRITICAL: "CRITICAL",
        StatusText.ERROR: "ERROR",
        StatusText.WARNING: "WARNING",
        StatusText.NOTICE: "NOTICE",
        StatusText.INFO: "INFO",
        StatusText.DEBUG: "DEBUG",
    }
    return names.get(level, f"S{level}")


def quaternion_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
