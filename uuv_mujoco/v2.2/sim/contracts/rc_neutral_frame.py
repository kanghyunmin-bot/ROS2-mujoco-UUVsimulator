"""Neutral MAVLink RC override frame construction."""

from __future__ import annotations

from .rc_constants import (
    NEUTRAL_DEFAULT_CHANNEL_COUNT,
    PWM_CENTER,
    RC_OVERRIDE_CHANNEL_COUNT,
    RC_RELEASE_VALUE,
)


def neutral_rc_override_frame(
    *,
    center_pwm: int = PWM_CENTER,
    channel_count: int = RC_OVERRIDE_CHANNEL_COUNT,
) -> list[int]:
    """Return the neutral MAVLink RC override frame used for SITL commands."""
    primary_count = min(NEUTRAL_DEFAULT_CHANNEL_COUNT, int(channel_count))
    extension_count = max(0, int(channel_count) - primary_count)
    return [int(center_pwm)] * primary_count + [RC_RELEASE_VALUE] * extension_count


__all__ = ["neutral_rc_override_frame"]
