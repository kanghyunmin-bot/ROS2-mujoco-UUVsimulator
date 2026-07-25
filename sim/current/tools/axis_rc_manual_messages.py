"""ManualControl message builders for axis checks."""

from __future__ import annotations

from mavros_msgs.msg import ManualControl

from axis_rc_command_values import clamp_axis_command


MANUAL_AXIS_FIELDS = {
    "forward": "x",
    "lateral": "y",
    "heave": "z",
    "yaw": "r",
}


def build_manual_control_message(axis: str | None = None, command: float = 0.0) -> ManualControl:
    msg = ManualControl()
    field = MANUAL_AXIS_FIELDS.get(axis or "")
    if field is not None:
        setattr(msg, field, clamp_axis_command(command))
    return msg


__all__ = ["MANUAL_AXIS_FIELDS", "build_manual_control_message"]
