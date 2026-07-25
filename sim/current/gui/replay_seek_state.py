"""Thread-safe seek state helpers for GUI RC replay."""

from __future__ import annotations

from .gui_axis_normalization import clamp
from .replay_format import format_replay_time


def _request_rc_replay_seek(self, time_s: float) -> None:
    time_s = clamp(float(time_s), 0.0, max(self._rc_replay_duration_s, 0.0))
    with self._rc_replay_seek_lock:
        self._rc_replay_seek_time_s = time_s
    if not self._rc_replay_running():
        self._set_rc_replay_position(time_s, force=True)
        self._set_rc_replay_status(f"replay: seek {format_replay_time(time_s)}")


def _consume_rc_replay_seek(self) -> float | None:
    with self._rc_replay_seek_lock:
        time_s = self._rc_replay_seek_time_s
        self._rc_replay_seek_time_s = None
    return time_s
