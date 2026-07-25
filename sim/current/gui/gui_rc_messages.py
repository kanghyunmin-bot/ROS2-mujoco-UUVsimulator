"""GUI RC override message construction helpers."""

from __future__ import annotations

from typing import Iterable

from sim.contracts import sanitize_primary_rc as contract_sanitize_primary_rc

from .config import (
    PRIMARY_RC_CHANNEL_COUNT,
    RC_HEAVE_CHANNEL_INDEX,
    RC_MESSAGE_CHANNEL_COUNT,
    RC_NEUTRAL_PWM,
    RcLayout,
)
from .gui_rc_padding import padded_rc_channels
from .gui_rc_pwm import axis_to_pwm, heave_axis_to_rc3_pwm
from .runtime import OverrideRCIn


def sanitize_primary_rc_override_channels(values: Iterable[int]) -> list[int]:
    """Return an 18-channel RC override frame with safe primary controls."""
    raw_values = list(values)
    channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
    for idx, value in enumerate(raw_values[:RC_MESSAGE_CHANNEL_COUNT]):
        channels[idx] = int(value)
    return contract_sanitize_primary_rc(
        channels,
        channel_count=RC_MESSAGE_CHANNEL_COUNT,
        center_pwm=RC_NEUTRAL_PWM,
    )


def make_rc_override_message(
    layout: RcLayout,
    *,
    yaw: float,
    heave: float,
    forward: float,
    lateral: float,
    invert_heave: bool = False,
) -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
    for idx in range(PRIMARY_RC_CHANNEL_COUNT):
        msg.channels[idx] = RC_NEUTRAL_PWM
    axes = {
        "heave": -heave if invert_heave else heave,
        "yaw": yaw,
        "forward": forward,
        "lateral": lateral,
    }
    for axis_name, axis_value in axes.items():
        channel_index = int(layout.axis_channels[axis_name])
        if axis_name == "heave" and channel_index == RC_HEAVE_CHANNEL_INDEX:
            msg.channels[channel_index] = heave_axis_to_rc3_pwm(axis_value)
        else:
            msg.channels[channel_index] = axis_to_pwm(axis_value)
    return msg


def make_rc_release_message() -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
    for idx in range(PRIMARY_RC_CHANNEL_COUNT):
        msg.channels[idx] = OverrideRCIn.CHAN_RELEASE
    return msg


__all__ = [
    "sanitize_primary_rc_override_channels",
    "make_rc_override_message",
    "make_rc_release_message",
    "padded_rc_channels",
]
