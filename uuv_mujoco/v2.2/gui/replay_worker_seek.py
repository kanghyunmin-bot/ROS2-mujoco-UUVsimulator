"""Seek handling for the GUI RC replay worker."""

from __future__ import annotations

import time

from .replay_format import format_replay_time
from .replay_worker_state import ReplayWorkerState


def consume_replay_seek(owner, state: ReplayWorkerState, *, rate: float, paused: bool = False) -> bool:
    seek_time_s = owner._consume_rc_replay_seek()
    if seek_time_s is None:
        return False
    state.idx = owner._rc_replay_sample_index_for_time(seek_time_s)
    state.start_wall = time.monotonic() - seek_time_s / rate
    if paused:
        state.paused_since = time.monotonic()
        owner._set_rc_replay_position(seek_time_s, force=False)
        owner._set_rc_replay_status(f"replay: paused at {format_replay_time(seek_time_s)}")
    else:
        owner._set_rc_replay_status(f"replay: seek {format_replay_time(seek_time_s)}")
    return True


__all__ = ["consume_replay_seek"]
