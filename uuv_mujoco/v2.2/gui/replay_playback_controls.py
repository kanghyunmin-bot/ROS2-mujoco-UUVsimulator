"""Start, pause, and stop actions for GUI RC replay."""

from __future__ import annotations

import threading

from .gui_axis_normalization import clamp


def _start_rc_replay(self) -> None:
    if self._rc_replay_running():
        return
    if not self._rc_replay_samples and not self._load_rc_replay():
        return

    _prepare_replay_start(self)
    rate = self._rc_replay_rate()
    start_time_s = _replay_start_time(self)
    samples = list(self._rc_replay_samples)
    self._rc_replay_thread = threading.Thread(
        target=self._run_rc_replay,
        args=(samples, rate, start_time_s),
        daemon=True,
    )
    self._rc_replay_thread.start()


def _toggle_rc_replay_pause(self) -> None:
    if not self._rc_replay_running():
        return
    if self._rc_replay_pause_event.is_set():
        self._rc_replay_pause_event.clear()
        self.rc_replay_pause_button.config(text="Pause")
        self._set_rc_replay_status("replay: running")
    else:
        self._rc_replay_pause_event.set()
        self.rc_replay_pause_button.config(text="Resume")
        self._set_rc_replay_status("replay: paused")


def _stop_rc_replay(self) -> None:
    running = self._rc_replay_running()
    self._rc_replay_stop_event.set()
    self._rc_replay_pause_event.clear()
    self.rc_replay_pause_button.config(text="Pause")
    self.node.publish_rc_release()
    self._guided_control_prev = False
    self._rc_override_prev = False
    self._set_rc_replay_status("replay: stopping" if running else "replay: stopped")


def _prepare_replay_start(self) -> None:
    self.rc_override_enabled.set(False)
    self._center_rc_sticks()
    self._rc_override_prev = False
    self._rc_replay_stop_event.clear()
    self._rc_replay_pause_event.clear()
    self.rc_replay_pause_button.config(text="Pause")


def _replay_start_time(self) -> float:
    start_time_s = clamp(
        float(self.rc_replay_position_var.get()),
        0.0,
        max(self._rc_replay_duration_s, 0.0),
    )
    if start_time_s >= max(self._rc_replay_duration_s - 0.01, 0.0):
        start_time_s = 0.0
        self._set_rc_replay_position(0.0, force=True)
    return start_time_s


__all__ = ["_start_rc_replay", "_stop_rc_replay", "_toggle_rc_replay_pause"]
