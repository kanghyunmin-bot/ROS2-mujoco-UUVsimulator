"""Pure formatting helpers for GUI RC replay."""

from __future__ import annotations

import math


def format_replay_time(time_s: float) -> str:
    if not math.isfinite(time_s):
        return "--:--"
    time_s = max(0.0, float(time_s))
    minutes = int(time_s // 60.0)
    seconds = time_s - minutes * 60.0
    return f"{minutes:02d}:{seconds:04.1f}"


__all__ = ["format_replay_time"]
