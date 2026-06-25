"""Pilot command reading and initial-depth release policy."""

from __future__ import annotations

from .config import AXIS_DEADBAND
from .gui_rc_axes import gui_rc_to_override_axes
from .models import ControlCommands


def read_control_commands(owner) -> ControlCommands:
    rc_forward, rc_lateral, rc_heave, rc_yaw = gui_rc_to_override_axes(
        forward=owner.rc_forward_var.get(),
        lateral=owner.rc_lateral_var.get(),
        heave=owner.rc_heave_var.get(),
        yaw=owner.rc_yaw_var.get(),
    )
    return ControlCommands(
        velocity_forward=0.0,
        velocity_lateral=0.0,
        velocity_heave=0.0,
        velocity_yaw=0.0,
        rc_forward=rc_forward,
        rc_lateral=rc_lateral,
        rc_heave=rc_heave,
        rc_yaw=rc_yaw,
    )


def request_initial_depth_release_for_pilot_input(owner, commands: ControlCommands) -> None:
    if getattr(owner, "_pilot_input_release_requested", False):
        return
    if not _has_nonzero_rc_axis(commands):
        return
    owner._pilot_input_release_requested = True
    owner.node.request_initial_depth_release_when_armed("pilot input")


def _has_nonzero_rc_axis(commands: ControlCommands) -> bool:
    values = (
        commands.rc_yaw,
        commands.rc_heave,
        commands.rc_forward,
        commands.rc_lateral,
    )
    return any(abs(float(value)) > AXIS_DEADBAND for value in values)


__all__ = ["read_control_commands", "request_initial_depth_release_for_pilot_input"]
