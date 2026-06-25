"""MAVLink telemetry status snapshot helpers."""

from __future__ import annotations

import math


TELEMETRY_STATUS_PREFIXES = (
    "heartbeat",
    "att",
    "lpos",
    "spress1",
    "spress2",
    "spress3",
    "rc_channels",
    "ekf",
    "vfr",
    "raw_imu",
    "scaled_imu",
    "statustext",
)


def build_status_snapshot(
    status_data: dict[str, object],
    now_wall: float,
    *,
    endpoint: str,
    requested_hz: float,
) -> dict[str, object]:
    status = dict(status_data)
    for prefix in TELEMETRY_STATUS_PREFIXES:
        wall_key = f"{prefix}_wall_s"
        try:
            wall_s = float(status.get(wall_key, float("nan")))
        except (TypeError, ValueError):
            wall_s = float("nan")
        if math.isfinite(wall_s):
            status[f"{prefix}_age_s"] = float(max(0.0, float(now_wall) - wall_s))
    status["active"] = bool(status)
    status["mavlink_endpoint"] = str(endpoint)
    status["requested_hz"] = float(requested_hz)
    return status


__all__ = ["TELEMETRY_STATUS_PREFIXES", "build_status_snapshot"]
