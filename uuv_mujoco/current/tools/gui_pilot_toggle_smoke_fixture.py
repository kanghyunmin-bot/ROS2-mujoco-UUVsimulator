"""Fixtures for GUI pilot-toggle smoke checks."""

from __future__ import annotations

from gui.models import ControlCommands


class BoolVar:
    def __init__(self, value: bool):
        self._value = value

    def get(self) -> bool:
        return self._value


class EventNode:
    def __init__(self):
        self.events: list[str] = []

    def push_event(self, text: str) -> None:
        self.events.append(text)


class PilotToggleOwner:
    def __init__(self, enabled: bool, replay_running: bool):
        self.rc_override_enabled = BoolVar(enabled)
        self.node = EventNode()
        self._replay_running = replay_running
        self._replay_stopped = False
        self._released = False
        self._published = False
        self._pilot_input_release_requested = True

    def _rc_replay_running(self) -> bool:
        return self._replay_running

    def _stop_rc_replay(self) -> None:
        self._replay_stopped = True

    def _read_control_commands(self) -> ControlCommands:
        return zero_control_commands()

    def _request_initial_depth_release_for_pilot_input(self, _commands: ControlCommands) -> None:
        self._pilot_input_release_requested = True

    def _publish_pilot_control(self, _commands: ControlCommands) -> None:
        self._published = True

    def _publish_rc_release_and_neutral(self) -> None:
        self._released = True


def zero_control_commands() -> ControlCommands:
    return ControlCommands(
        velocity_forward=0.0,
        velocity_lateral=0.0,
        velocity_heave=0.0,
        velocity_yaw=0.0,
        rc_forward=0.0,
        rc_lateral=0.0,
        rc_heave=0.0,
        rc_yaw=0.0,
    )


__all__ = ["PilotToggleOwner", "zero_control_commands"]
