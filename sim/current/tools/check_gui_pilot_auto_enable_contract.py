#!/usr/bin/env python3
"""Regression check for GUI stick-change pilot auto-enable behavior."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.control_pilot_mixin import ControlPilotMixin  # noqa: E402
from gui.models import ControlCommands  # noqa: E402


class BoolVar:
    def __init__(self, value: bool):
        self._value = value

    def get(self) -> bool:
        return self._value

    def set(self, value: bool) -> None:
        self._value = bool(value)


class AutoEnableOwner(ControlPilotMixin):
    def __init__(self, *, enabled: bool, commands: ControlCommands):
        self.rc_override_enabled = BoolVar(enabled)
        self.commands = commands
        self.toggle_calls = 0
        self.publish_calls = 0
        self.release_calls = 0
        self.depth_release_calls = 0

    def _read_control_commands(self) -> ControlCommands:
        return self.commands

    def _on_rc_override_toggle(self) -> None:
        self.toggle_calls += 1

    def _request_initial_depth_release_for_pilot_input(self, _commands: ControlCommands) -> None:
        self.depth_release_calls += 1

    def _publish_pilot_control(self, _commands: ControlCommands) -> None:
        self.publish_calls += 1


class EventNode:
    def __init__(self):
        self.events: list[str] = []

    def push_event(self, message: str) -> None:
        self.events.append(message)


class RealToggleAutoEnableOwner(ControlPilotMixin):
    def __init__(self, *, commands: ControlCommands):
        self.rc_override_enabled = BoolVar(False)
        self.commands = commands
        self.node = EventNode()
        self.publish_calls = 0
        self.depth_release_calls = 0
        self.replay_stopped = False
        self._pilot_input_release_requested = False

    def _read_control_commands(self) -> ControlCommands:
        return self.commands

    def _rc_replay_running(self) -> bool:
        return False

    def _stop_rc_replay(self) -> None:
        self.replay_stopped = True

    def _request_initial_depth_release_for_pilot_input(self, _commands: ControlCommands) -> None:
        self.depth_release_calls += 1
        self._pilot_input_release_requested = True

    def _publish_pilot_control(self, _commands: ControlCommands) -> None:
        self.publish_calls += 1

    def _publish_rc_release_and_neutral(self) -> None:
        raise AssertionError("auto-enable path must not release RC override")


def commands(*, yaw: float = 0.0, heave: float = 0.0, forward: float = 0.0, lateral: float = 0.0) -> ControlCommands:
    return ControlCommands(
        velocity_forward=0.0,
        velocity_lateral=0.0,
        velocity_heave=0.0,
        velocity_yaw=0.0,
        rc_forward=forward,
        rc_lateral=lateral,
        rc_heave=heave,
        rc_yaw=yaw,
    )


def main() -> int:
    neutral = AutoEnableOwner(enabled=False, commands=commands())
    neutral._on_rc_stick_changed()
    assert not neutral.rc_override_enabled.get(), "neutral stick must not auto-enable pilot input"
    assert neutral.toggle_calls == 0, "neutral stick must not call toggle"
    assert neutral.publish_calls == 0, "neutral stick must not publish"

    yaw = AutoEnableOwner(enabled=False, commands=commands(yaw=0.2))
    yaw._on_rc_stick_changed()
    assert yaw.rc_override_enabled.get(), "non-neutral stick must auto-enable pilot input"
    assert yaw.toggle_calls == 1, "auto-enable must run the normal toggle flow"
    assert yaw.publish_calls == 0, "toggle flow owns the first publish after auto-enable"

    real_toggle = RealToggleAutoEnableOwner(commands=commands(yaw=0.2))
    real_toggle._on_rc_stick_changed()
    assert real_toggle.rc_override_enabled.get(), "real toggle path must enable pilot input"
    assert real_toggle.depth_release_calls == 1, "real toggle path must request initial-depth release"
    assert real_toggle.publish_calls == 1, "real toggle path must publish first stick command immediately"
    assert any("pilot control enabled" in event for event in real_toggle.node.events), "real toggle event missing"

    already_enabled = AutoEnableOwner(enabled=True, commands=commands(forward=0.2))
    already_enabled._on_rc_stick_changed()
    assert already_enabled.toggle_calls == 0, "enabled stick change must not re-toggle"
    assert already_enabled.depth_release_calls == 1, "enabled stick change must request initial-depth release"
    assert already_enabled.publish_calls == 1, "enabled stick change must publish immediately"

    print("gui_pilot_auto_enable_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
