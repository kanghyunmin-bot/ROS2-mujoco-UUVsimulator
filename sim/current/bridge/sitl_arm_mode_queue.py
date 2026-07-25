"""Arm/mode queue entry points for SitlTransport."""

from __future__ import annotations

from .sitl_arm_mode_queue_arm import queue_arm_command_impl
from .sitl_arm_mode_queue_mode import queue_set_mode_impl


def queue_arm_command(self, arm: bool) -> bool:
    return queue_arm_command_impl(self, arm)


def queue_set_mode(self, mode: str) -> bool:
    return queue_set_mode_impl(self, mode)


def send_arm_command(self, arm: bool) -> bool:
    """Forward arm/disarm request to ArduSub SITL over MAVLink."""
    return self.queue_arm_command(bool(arm))


def send_set_mode(self, mode: str) -> bool:
    """Forward custom mode request to ArduSub SITL over MAVLink."""
    return self.queue_set_mode(str(mode))


__all__ = ["queue_arm_command", "queue_set_mode", "send_arm_command", "send_set_mode"]
