"""ArduSub RC override marker normalization."""

from __future__ import annotations

from .rc_constants import (
    NEUTRAL_DEFAULT_CHANNEL_COUNT,
    RC_OVERRIDE_CHANNEL_COUNT,
)
from .rc_frame_fill import pad_override_values
from .rc_value_rules import normalize_extension_override_value, normalize_legacy_override_value


def normalize_ardusub_rc_override(
    values: list[int],
    *,
    channel_count: int = RC_OVERRIDE_CHANNEL_COUNT,
) -> list[int]:
    """Normalize MAVROS-style RC override for local ArduSub 4.1.x handling."""
    out = pad_override_values(values, channel_count=int(channel_count))
    normalized_values: list[int] = []
    for idx, raw in enumerate(out[: int(channel_count)]):
        value = int(raw)
        if idx < NEUTRAL_DEFAULT_CHANNEL_COUNT:
            normalized_values.append(normalize_legacy_override_value(value))
        else:
            normalized_values.append(normalize_extension_override_value(value))
    return normalized_values


__all__ = ["normalize_ardusub_rc_override"]
