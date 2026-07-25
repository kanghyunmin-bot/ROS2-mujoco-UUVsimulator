"""Thread-safe status helpers for GUI RC replay."""

from __future__ import annotations

import threading


def _set_rc_replay_status(self, text: str) -> None:
    if threading.current_thread() is threading.main_thread():
        self.rc_replay_status_var.set(text)
        return
    try:
        self.root.after(0, lambda: self.rc_replay_status_var.set(text))
    except Exception:
        pass


def _set_rc_replay_pause_button(self, text: str) -> None:
    try:
        self.root.after(0, lambda: self.rc_replay_pause_button.config(text=text))
    except Exception:
        pass


def _rc_replay_running(self) -> bool:
    return self._rc_replay_thread is not None and self._rc_replay_thread.is_alive()
