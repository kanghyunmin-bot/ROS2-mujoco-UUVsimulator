"""RC and MANUAL_CONTROL publishing helpers for axis RC checks."""

from __future__ import annotations

from axis_rc_node_publish import (
    publish_manual_control,
    publish_neutral_control,
    publish_rc_override,
    publish_rc_release,
)
from axis_rc_node_spin import spin_with_axis_rc


class AxisRcNodeControl:
    def publish_rc(self, axis: str | None = None, command: float = 0.0) -> None:
        publish_rc_override(self, axis, command)

    def publish_manual(self, axis: str | None = None, command: float = 0.0) -> None:
        publish_manual_control(self, axis, command)

    def release_rc(self) -> None:
        publish_rc_release(self)

    def publish_neutral_control(self) -> None:
        publish_neutral_control(self)

    def spin_with_rc(
        self,
        duration: float,
        axis: str | None = None,
        command: float = 0.0,
        hz: float = 100.0,
        input_mode: str = "rc-override",
    ) -> None:
        spin_with_axis_rc(self, duration, axis=axis, command=command, hz=hz, input_mode=input_mode)


__all__ = ["AxisRcNodeControl"]
