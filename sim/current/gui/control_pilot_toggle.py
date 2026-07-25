"""Pilot-control toggle flow for the GUI."""

from __future__ import annotations

from .config import GUI_PILOT_CONTROL_MODE


def handle_rc_override_toggle(owner) -> None:
    if owner.rc_override_enabled.get() and owner._rc_replay_running():
        owner._stop_rc_replay()
    if owner.rc_override_enabled.get():
        _enable_pilot_control(owner)
    else:
        _release_pilot_control(owner)


def _enable_pilot_control(owner) -> None:
    commands = owner._read_control_commands()
    owner._pilot_input_release_requested = False
    owner._request_initial_depth_release_for_pilot_input(commands)
    owner._publish_pilot_control(commands)
    owner.node.push_event(f"pilot control enabled ({GUI_PILOT_CONTROL_MODE})")


def _release_pilot_control(owner) -> None:
    owner._publish_rc_release_and_neutral()
    owner.node.push_event("pilot control released")


__all__ = ["handle_rc_override_toggle"]
