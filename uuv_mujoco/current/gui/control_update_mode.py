"""Control-mode labels for the GUI update loop."""

from __future__ import annotations


def active_control_mode(owner) -> str:
    if owner.rc_override_enabled.get():
        return "pilot control"
    if owner._rc_replay_running():
        return "RC replay"
    return "idle"


__all__ = ["active_control_mode"]
