"""Pilot command publication policy."""

from __future__ import annotations

from .config import GUI_PILOT_CONTROL_MODE, PILOT_CONTROL_RC_OVERRIDE
from .models import ControlCommands


def publish_pilot_control(owner, commands: ControlCommands) -> None:
    if GUI_PILOT_CONTROL_MODE == PILOT_CONTROL_RC_OVERRIDE:
        owner.node.publish_rc_override(
            yaw=commands.rc_yaw,
            heave=commands.rc_heave,
            forward=commands.rc_forward,
            lateral=commands.rc_lateral,
        )
        return
    owner.node.publish_manual_control(
        yaw=commands.rc_yaw,
        heave=commands.rc_heave,
        forward=commands.rc_forward,
        lateral=commands.rc_lateral,
    )


def publish_active_controls(owner, commands: ControlCommands) -> None:
    rc_active = owner.rc_override_enabled.get()

    if rc_active:
        owner._request_initial_depth_release_for_pilot_input(commands)
        owner._publish_pilot_control(commands)
    elif owner._rc_override_prev:
        owner._publish_rc_release_and_neutral()
    owner._rc_override_prev = rc_active


__all__ = ["publish_active_controls", "publish_pilot_control"]
