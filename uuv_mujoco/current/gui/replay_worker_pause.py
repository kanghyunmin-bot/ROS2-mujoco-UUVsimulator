"""Pause handling for the GUI RC replay worker."""

from __future__ import annotations

import time

from .replay_worker_seek import consume_replay_seek
from .replay_worker_state import ReplayWorkerState


def handle_replay_pause(owner, state: ReplayWorkerState, *, rate: float) -> None:
    while owner._rc_replay_pause_event.is_set() and not owner._rc_replay_stop_event.is_set():
        if state.paused_since is None:
            state.paused_since = time.monotonic()
        consume_replay_seek(owner, state, rate=rate, paused=True)
        time.sleep(0.03)
    if state.paused_since is not None:
        state.start_wall += time.monotonic() - state.paused_since
        state.paused_since = None


__all__ = ["handle_replay_pause"]
