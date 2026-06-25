"""ExternalNav readiness rules for the GUI."""

from __future__ import annotations

from .config import BACKEND_SIM_BRIDGE
from .models import TelemetrySnapshot


def sitl_extnav_ready(backend: str, snap: TelemetrySnapshot) -> bool:
    """Return whether the required ExternalNav stream is ready."""
    if backend != BACKEND_SIM_BRIDGE:
        return True
    if not bool(snap.sitl_extnav_required):
        return True
    return bool(snap.sitl_extnav_ready)


__all__ = ["sitl_extnav_ready"]
