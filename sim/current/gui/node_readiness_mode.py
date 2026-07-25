"""Required-mode helpers for GUI command-readiness checks."""

from __future__ import annotations

from .config import BACKEND_SIM_BRIDGE
from .models import TelemetrySnapshot


def required_auto_ready_mode(backend: str, snap: TelemetrySnapshot) -> str:
    if backend == BACKEND_SIM_BRIDGE and bool(snap.sitl_mavlink_status.get("auto_ready_enabled", False)):
        if bool(snap.sitl_mavlink_status.get("auto_ready_done", False)):
            return ""
        return str(snap.sitl_mavlink_status.get("auto_ready_mode", "MANUAL") or "MANUAL")
    return ""


__all__ = ["required_auto_ready_mode"]
