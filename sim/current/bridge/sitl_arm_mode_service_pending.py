"""Pending arm/mode retry service loops for SitlTransport."""

from __future__ import annotations

from .sitl_arm_mode_service_arm import service_pending_arm_command
from .sitl_arm_mode_service_mode import service_pending_mode_command


def _service_pending_arm_command(self, now_wall: float) -> None:
    service_pending_arm_command(self, now_wall)


def _service_pending_mode_command(self, now_wall: float) -> None:
    service_pending_mode_command(self, now_wall)


__all__ = ["_service_pending_arm_command", "_service_pending_mode_command"]
