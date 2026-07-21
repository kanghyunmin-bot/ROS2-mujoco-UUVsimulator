"""SITL command-link readiness rules for the GUI."""

from __future__ import annotations

import math

from .config import BACKEND_SIM_BRIDGE
from .models import TelemetrySnapshot


FRESH_COMMAND_STATUS_S = 3.0


def _fresh(age_s: float, *, max_age_s: float = FRESH_COMMAND_STATUS_S) -> bool:
    return math.isfinite(age_s) and age_s < max_age_s


def sitl_mavlink_command_alive(backend: str, snap: TelemetrySnapshot) -> bool:
    """Return whether the SITL command path is usable for arm/mode/RC."""
    if backend != BACKEND_SIM_BRIDGE:
        return True
    status_fresh = _fresh(snap.sitl_mavlink_status_age_s)
    heartbeat_fresh = _fresh(snap.sitl_mavlink_heartbeat_age_s)
    command_heartbeat_fresh = _fresh(snap.sitl_mavlink_command_heartbeat_age_s)
    if not command_heartbeat_fresh:
        command_heartbeat_fresh = heartbeat_fresh
    if snap.sitl_mavlink_rc_override_ready and status_fresh:
        return bool(snap.sitl_mavlink_active)
    return bool(
        snap.sitl_mavlink_active
        and status_fresh
        and command_heartbeat_fresh
        and snap.sitl_mavlink_rc_override_ready
    )


__all__ = ["FRESH_COMMAND_STATUS_S", "sitl_mavlink_command_alive"]
