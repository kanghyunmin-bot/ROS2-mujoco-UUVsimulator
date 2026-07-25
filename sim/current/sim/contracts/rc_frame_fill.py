"""Frame padding/default helpers for RC override contracts."""

from __future__ import annotations

from .rc_constants import (
    NEUTRAL_DEFAULT_CHANNEL_COUNT,
    RC_IGNORE_VALUE,
    RC_RELEASE_VALUE,
)


def default_sanitized_channel_value(index: int, *, center_pwm: int) -> int:
    return int(center_pwm) if int(index) < NEUTRAL_DEFAULT_CHANNEL_COUNT else RC_RELEASE_VALUE


def pad_override_values(values: list[int], *, channel_count: int) -> list[int]:
    target_count = int(channel_count)
    out = list(values[:target_count])
    if len(out) >= target_count:
        return out
    missing_primary = max(0, NEUTRAL_DEFAULT_CHANNEL_COUNT - len(out))
    missing_extension = max(0, target_count - max(len(out), NEUTRAL_DEFAULT_CHANNEL_COUNT))
    out.extend([RC_IGNORE_VALUE] * missing_primary)
    out.extend([RC_RELEASE_VALUE] * missing_extension)
    return out


__all__ = ["default_sanitized_channel_value", "pad_override_values"]
