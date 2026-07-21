"""Pure RC channel padding helpers."""

from __future__ import annotations

from typing import Iterable


RC_CHAN_RELEASE = 0
RC_CHAN_NOCHANGE = 65535


def padded_rc_channels(
    values: Iterable[int],
    *,
    target_count: int,
    sanitize_override_markers: bool = False,
) -> list[int]:
    channels: list[int] = []
    for value in list(values)[:target_count]:
        ivalue = int(value)
        if sanitize_override_markers and ivalue in (RC_CHAN_NOCHANGE, RC_CHAN_RELEASE):
            ivalue = 0
        elif sanitize_override_markers and (ivalue < 800 or ivalue > 2200):
            ivalue = 0
        channels.append(ivalue)
    if len(channels) < target_count:
        channels.extend([0] * (target_count - len(channels)))
    return channels


__all__ = ["RC_CHAN_NOCHANGE", "RC_CHAN_RELEASE", "padded_rc_channels"]
