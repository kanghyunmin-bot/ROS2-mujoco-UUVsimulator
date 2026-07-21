"""Timeline, seek, and rate helpers for GUI RC replay."""

from __future__ import annotations

import threading

from .gui_axis_normalization import clamp
from .replay_seek_state import _consume_rc_replay_seek, _request_rc_replay_seek
from .replay_slider_events import (
    _event_to_rc_replay_time,
    _on_rc_replay_slider_changed,
    _on_rc_replay_slider_motion,
    _on_rc_replay_slider_press,
    _on_rc_replay_slider_release,
    _set_replay_slider_from_event,
)
from .replay_time_math import (
    normalized_replay_rate,
    replay_sample_index_for_time,
    replay_time_label,
)


def _rc_replay_sample_index_for_time(self, time_s: float) -> int:
    return replay_sample_index_for_time(
        self._rc_replay_samples,
        self._rc_replay_duration_s,
        time_s,
    )


def _set_rc_replay_position(self, time_s: float, *, force: bool = False) -> None:
    time_s = clamp(float(time_s), 0.0, max(self._rc_replay_duration_s, 0.0))

    def apply() -> None:
        if self._rc_replay_slider_dragging and not force:
            return
        self.rc_replay_position_var.set(time_s)
        self._update_rc_replay_time_label(time_s)

    if threading.current_thread() is threading.main_thread():
        apply()
    else:
        try:
            self.root.after(0, apply)
        except Exception:
            pass


def _update_rc_replay_time_label(self, time_s: float | None = None) -> None:
    if time_s is None:
        try:
            time_s = float(self.rc_replay_position_var.get())
        except Exception:
            time_s = 0.0
    self.rc_replay_time_var.set(replay_time_label(time_s, self._rc_replay_duration_s))


def _rc_replay_rate(self) -> float:
    rate = normalized_replay_rate(self.rc_replay_rate_var.get())
    self.rc_replay_rate_var.set(f"{rate:g}")
    return rate
