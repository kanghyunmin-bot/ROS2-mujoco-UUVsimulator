"""GUI RC frame normalization helpers."""

from __future__ import annotations

from .config_rc import RC_FEEDBACK_CHANNEL_COUNT


CHAN_RELEASE = 0
CHAN_NOCHANGE = 65535


def padded_rc_channels(
    values,
    *,
    target_count: int = RC_FEEDBACK_CHANNEL_COUNT,
    sanitize_override_markers: bool = False,
) -> list[int]:
    channels: list[int] = []
    for value in list(values)[:target_count]:
        channels.append(
            sanitized_gui_rc_value(
                int(value),
                sanitize_override_markers=sanitize_override_markers,
            )
        )
    if len(channels) < target_count:
        channels.extend([0] * (target_count - len(channels)))
    return channels


def sanitized_gui_rc_value(value: int, *, sanitize_override_markers: bool) -> int:
    if not sanitize_override_markers:
        return int(value)
    if value in (CHAN_NOCHANGE, CHAN_RELEASE):
        return 0
    if value < 800 or value > 2200:
        return 0
    return int(value)


__all__ = [
    "CHAN_NOCHANGE",
    "CHAN_RELEASE",
    "padded_rc_channels",
    "sanitized_gui_rc_value",
]
