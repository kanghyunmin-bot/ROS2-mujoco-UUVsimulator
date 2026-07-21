"""Pure time and rate helpers for GUI RC replay."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from .gui_axis_normalization import clamp
from .replay_format import format_replay_time


def replay_sample_index_for_time(samples: Sequence[Any], duration_s: float, time_s: float) -> int:
    """Return the first sample index whose timestamp is at or after ``time_s``."""

    if not samples:
        return 0
    target = clamp(float(time_s), 0.0, float(duration_s))
    lo = 0
    hi = len(samples)
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid].time_s < target:
            lo = mid + 1
        else:
            hi = mid
    return min(lo, len(samples) - 1)


def replay_time_label(time_s: float, duration_s: float) -> str:
    return f"{format_replay_time(time_s)} / {format_replay_time(duration_s)}"


def replay_time_from_event_x(event_x: float, width: int, duration_s: float) -> float:
    width = max(int(width), 1)
    ratio = clamp(float(event_x) / float(width), 0.0, 1.0)
    return ratio * max(float(duration_s), 0.0)


def normalized_replay_rate(raw_rate: object) -> float:
    try:
        rate = float(raw_rate)
    except (TypeError, ValueError):
        rate = 1.0
    return clamp(rate, 0.1, 5.0)
