"""MAVROS command helpers for the roll stability probe."""

from __future__ import annotations

from roll_stability_command_send import arm_via_mavros, set_mode_via_mavros
from roll_stability_command_wait import wait_for_stack_ready


class RollStabilityCommandMixin:
    def wait_for_stack(self, timeout: float = 75.0) -> None:
        wait_for_stack_ready(self, timeout=timeout)

    def set_mode(self, mode: str, timeout: float = 8.0) -> None:
        set_mode_via_mavros(self, mode, timeout=timeout)

    def arm(self, value: bool, timeout: float = 8.0) -> None:
        arm_via_mavros(self, value, timeout=timeout)


__all__ = ["RollStabilityCommandMixin"]
