"""Load action for GUI RC replay samples."""

from __future__ import annotations

from .rc_replay_loader import load_rc_override_replay
from .replay_format import format_replay_time


def _load_rc_replay(self) -> bool:
    if self._rc_replay_running():
        self._set_rc_replay_status("replay: stop current playback before loading")
        return False
    try:
        samples = load_rc_override_replay(self.rc_replay_path_var.get())
    except Exception as exc:
        _clear_replay_load_state(self)
        self._set_rc_replay_status(f"replay load failed: {exc}")
        return False

    self._rc_replay_samples = samples
    duration = samples[-1].time_s if samples else 0.0
    self._rc_replay_duration_s = duration
    self.rc_replay_slider.configure(to=max(duration, 1e-6))
    self.rc_replay_slider.state(["!disabled"])
    self._set_rc_replay_position(0.0, force=True)
    with self._rc_replay_seek_lock:
        self._rc_replay_seek_time_s = None
    self._set_rc_replay_status(
        f"replay loaded: {len(samples)} msgs, {format_replay_time(duration)}"
    )
    return True


def _clear_replay_load_state(self) -> None:
    self._rc_replay_samples = []
    self._rc_replay_duration_s = 0.0
    self.rc_replay_slider.configure(to=1.0)
    self.rc_replay_slider.state(["disabled"])
    self._set_rc_replay_position(0.0, force=True)


__all__ = ["_load_rc_replay"]
