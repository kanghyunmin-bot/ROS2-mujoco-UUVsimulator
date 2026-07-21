"""Event list and RC feedback widget updates."""

from __future__ import annotations

from collections import deque
from typing import Deque

from .config import RC_VISIBLE_CHANNEL_COUNT
from .helpers import clamp
from .runtime import tk


class ControlFeedbackMixin:
    def _update_events(self, events: Deque[str] | deque[str]) -> None:
        top = events[0] if events else ""
        if top == self._last_event_top:
            return
        self._last_event_top = top
        self.event_list.delete(0, tk.END)
        for item in events:
            self.event_list.insert(tk.END, item)

    def _update_rc_feedback_bars(self, channels: list[int]) -> None:
        for idx, channel in enumerate(channels[:RC_VISIBLE_CHANNEL_COUNT]):
            value = int(channel)
            self._rc_bars[idx]["value"] = clamp(value - 1100, 0, 800) if value > 0 else 0
            self._rc_labels[idx].config(text=str(value))

    @staticmethod
    def _has_rc_feedback(channels: list[int]) -> bool:
        return any(int(value) > 0 for value in channels[:RC_VISIBLE_CHANNEL_COUNT])
