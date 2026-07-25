"""GUI/replay RC override sanitization."""

from __future__ import annotations

from .rc_constants import (
    NEUTRAL_DEFAULT_CHANNEL_COUNT,
    PRIMARY_RC_CHANNEL_COUNT,
    PWM_CENTER,
    RC_OVERRIDE_CHANNEL_COUNT,
)
from .rc_frame_fill import default_sanitized_channel_value
from .rc_value_rules import (
    sanitize_extension_value,
    sanitize_legacy_override_value,
    sanitize_primary_motion_value,
)


def _sanitize_channel(idx: int, value: int, *, center_pwm: int) -> int:
    if idx < PRIMARY_RC_CHANNEL_COUNT:
        return sanitize_primary_motion_value(value, center_pwm=center_pwm)
    if idx < NEUTRAL_DEFAULT_CHANNEL_COUNT:
        return sanitize_legacy_override_value(value, center_pwm=center_pwm)
    return sanitize_extension_value(value)


def sanitize_primary_rc(
    values: list[int],
    *,
    channel_count: int = RC_OVERRIDE_CHANNEL_COUNT,
    center_pwm: int = PWM_CENTER,
) -> list[int]:
    """Preserve the MAVLink2 18-channel shape while protecting motion inputs."""
    out: list[int] = []
    for idx in range(channel_count):
        default = default_sanitized_channel_value(idx, center_pwm=center_pwm)
        value = int(values[idx]) if idx < len(values) else default
        out.append(_sanitize_channel(idx, value, center_pwm=center_pwm))
    return out


__all__ = ["sanitize_primary_rc"]
