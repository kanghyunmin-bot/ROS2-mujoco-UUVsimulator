"""Frame extraction and normalized-axis math for MAVROS RC override input."""

from __future__ import annotations

from collections.abc import Sequence

from .ros2_math import rc_channel_value, rc_to_norm

MAX_RC_OVERRIDE_CHANNELS = 18


def rc_override_channels_from_msg(msg) -> Sequence[int] | None:
    channels = getattr(msg, "channels", None)
    if channels is None or len(channels) == 0:
        return None
    return channels


def rc_override_forward_frame(channels: Sequence[int]) -> list[int]:
    return [int(v) for v in channels[:MAX_RC_OVERRIDE_CHANNELS]]


def normalized_rc_override_axes(self, channels: Sequence[int]) -> tuple[float, float, float, float]:
    fwd = rc_to_norm(
        rc_channel_value(channels, self._mavros_rc_forward_channel),
        pwm_span=self._mavros_rc_pwm_span,
        invert=self._mavros_rc_forward_invert,
    )
    sway = rc_to_norm(
        rc_channel_value(channels, self._mavros_rc_sway_channel),
        pwm_span=self._mavros_rc_pwm_span,
        invert=self._mavros_rc_sway_invert,
    )
    yaw = rc_to_norm(
        rc_channel_value(channels, self._mavros_rc_yaw_channel),
        pwm_span=self._mavros_rc_pwm_span,
        invert=self._mavros_rc_yaw_invert,
    )
    heave = rc_to_norm(
        rc_channel_value(channels, self._mavros_rc_heave_channel),
        pwm_span=self._mavros_rc_pwm_span,
        invert=self._mavros_rc_heave_invert,
    )
    return fwd, sway, yaw, heave


__all__ = [
    "MAX_RC_OVERRIDE_CHANNELS",
    "normalized_rc_override_axes",
    "rc_override_channels_from_msg",
    "rc_override_forward_frame",
]
