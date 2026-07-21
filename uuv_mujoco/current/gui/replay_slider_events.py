"""Tk slider event handlers for GUI RC replay."""

from __future__ import annotations

from .replay_time_math import replay_time_from_event_x


def _event_to_rc_replay_time(self, event) -> float:
    width = self.rc_replay_slider.winfo_width()
    return replay_time_from_event_x(event.x, width, self._rc_replay_duration_s)


def _set_replay_slider_from_event(self, event) -> float:
    time_s = self._event_to_rc_replay_time(event)
    self.rc_replay_position_var.set(time_s)
    self._update_rc_replay_time_label(time_s)
    return time_s


def _on_rc_replay_slider_changed(self, value: str) -> None:
    if self._rc_replay_slider_dragging:
        return
    try:
        time_s = float(value)
    except (TypeError, ValueError):
        return
    self._update_rc_replay_time_label(time_s)


def _on_rc_replay_slider_press(self, event):
    if not self._rc_replay_samples:
        return "break"
    self._rc_replay_slider_dragging = True
    self._set_replay_slider_from_event(event)
    return "break"


def _on_rc_replay_slider_motion(self, event):
    if not self._rc_replay_samples:
        return "break"
    self._set_replay_slider_from_event(event)
    return "break"


def _on_rc_replay_slider_release(self, event):
    if not self._rc_replay_samples:
        self._rc_replay_slider_dragging = False
        return "break"
    time_s = self._set_replay_slider_from_event(event)
    self._rc_replay_slider_dragging = False
    self._request_rc_replay_seek(time_s)
    return "break"
