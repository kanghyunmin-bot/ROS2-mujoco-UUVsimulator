"""Pilot command and RC override publishing methods for the GUI."""

from __future__ import annotations

from .control_pilot_commands import (
    read_control_commands,
    request_initial_depth_release_for_pilot_input,
)
from .control_pilot_publish import publish_active_controls, publish_pilot_control
from .control_pilot_release import (
    center_rc_sticks,
    publish_rc_release_and_neutral,
    release_rc_override,
)
from .control_pilot_toggle import handle_rc_override_toggle
from .config import AXIS_DEADBAND
from .models import ControlCommands


class ControlPilotMixin:
    def _zero_controls(self) -> None:
        self._center_rc_sticks()

    def _center_rc_sticks(self) -> None:
        center_rc_sticks(self)

    def _release_rc_override(self) -> None:
        release_rc_override(self)

    def _publish_rc_release_and_neutral(self) -> None:
        publish_rc_release_and_neutral(self)

    def _publish_pilot_control(self, commands: ControlCommands) -> None:
        publish_pilot_control(self, commands)

    def _on_rc_override_toggle(self) -> None:
        handle_rc_override_toggle(self)

    def _read_control_commands(self) -> ControlCommands:
        return read_control_commands(self)

    def _publish_active_controls(self, commands: ControlCommands) -> None:
        publish_active_controls(self, commands)

    def _on_rc_stick_changed(self) -> None:
        commands = self._read_control_commands()
        if not self.rc_override_enabled.get():
            if not _has_nonzero_rc_axis(commands):
                return
            self.rc_override_enabled.set(True)
            self._on_rc_override_toggle()
            return
        self._request_initial_depth_release_for_pilot_input(commands)
        self._publish_pilot_control(commands)

    def _request_initial_depth_release_for_pilot_input(self, commands: ControlCommands) -> None:
        request_initial_depth_release_for_pilot_input(self, commands)


def _has_nonzero_rc_axis(commands: ControlCommands) -> bool:
    return any(
        abs(float(value)) > AXIS_DEADBAND
        for value in (
            commands.rc_yaw,
            commands.rc_heave,
            commands.rc_forward,
            commands.rc_lateral,
        )
    )
