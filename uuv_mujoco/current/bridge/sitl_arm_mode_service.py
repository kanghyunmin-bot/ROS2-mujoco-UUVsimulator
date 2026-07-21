"""Compatibility exports for pending arm/mode retry service loops."""

from __future__ import annotations

from .sitl_arm_mode_service_pending import (
    _service_pending_arm_command,
    _service_pending_mode_command,
)


__all__ = ["_service_pending_arm_command", "_service_pending_mode_command"]
