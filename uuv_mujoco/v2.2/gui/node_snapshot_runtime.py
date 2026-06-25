"""Telemetry snapshot, event, and payload helpers for UuvGuiNode."""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import replace
from typing import Any

from .config import TELEMETRY_EVENT_LIMIT
from .models import TelemetrySnapshot


def payload_float(payload: dict, key: str, default: float = math.inf) -> float:
    try:
        value = float(payload.get(key, default))
    except (TypeError, ValueError):
        return float(default)
    return value if math.isfinite(value) else float(default)


def touch(owner: Any, key: str) -> None:
    owner._last_wall[key] = time.monotonic()


def push_event(owner: Any, text: str) -> None:
    stamp = time.strftime("%H:%M:%S")
    with owner._lock:
        owner._snapshot.events.appendleft(f"[{stamp}] {text}")


def snapshot(owner: Any) -> TelemetrySnapshot:
    now = time.monotonic()
    with owner._lock:
        snap = replace(
            owner._snapshot,
            rc_in=list(owner._snapshot.rc_in),
            rc_out=list(owner._snapshot.rc_out),
            sitl_mavlink_status=dict(owner._snapshot.sitl_mavlink_status),
            events=deque(owner._snapshot.events, maxlen=TELEMETRY_EVENT_LIMIT),
        )

    snap.state_age_s = now - owner._last_wall.get("state", math.inf)
    snap.imu_age_s = now - owner._last_wall.get("imu", math.inf)
    snap.pose_age_s = now - owner._last_wall.get("pose", math.inf)
    snap.depth_age_s = now - owner._last_wall.get("depth", math.inf)
    snap.rc_in_age_s = now - owner._last_wall.get("rc_in", math.inf)
    snap.rc_out_age_s = now - owner._last_wall.get("rc_out", math.inf)
    snap.rc_age_s = min(snap.rc_in_age_s, snap.rc_out_age_s)
    snap.ping360_age_s = now - owner._last_wall.get("ping360", math.inf)
    snap.real_start_age_s = now - owner._last_wall.get("real_start", math.inf)
    snap.sitl_mavlink_status_age_s = now - owner._last_wall.get("sitl_mavlink_status", math.inf)
    return snap


__all__ = ["payload_float", "push_event", "snapshot", "touch"]
