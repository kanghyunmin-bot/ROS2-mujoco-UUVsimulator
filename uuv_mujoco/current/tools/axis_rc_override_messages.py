"""OverrideRCIn message builders for axis checks."""

from __future__ import annotations

from mavros_msgs.msg import OverrideRCIn

from axis_rc_command_values import axis_command_value
from axis_rc_contract import AXIS_TO_CHANNEL, RC_NEUTRAL, RC_SPAN


def build_rc_override_message(
    axis: str | None = None,
    command: float = 0.0,
    *,
    invert_heave_rc: bool = False,
) -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = _neutral_primary_rc_frame()
    if axis:
        _apply_axis_override(msg, axis=axis, command=command, invert_heave_rc=invert_heave_rc)
    return msg


def build_rc_release_message() -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
    for idx in range(8):
        msg.channels[idx] = OverrideRCIn.CHAN_RELEASE
    return msg


def _neutral_primary_rc_frame() -> list[int]:
    channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
    for idx in range(8):
        channels[idx] = RC_NEUTRAL
    return channels


def _apply_axis_override(
    msg: OverrideRCIn,
    *,
    axis: str,
    command: float,
    invert_heave_rc: bool,
) -> None:
    channel = AXIS_TO_CHANNEL[axis]
    command_value = axis_command_value(axis, command, invert_heave_rc=invert_heave_rc)
    msg.channels[channel] = int(round(RC_NEUTRAL + RC_SPAN * command_value))


__all__ = ["build_rc_override_message", "build_rc_release_message"]
