"""Sample timing wait for the GUI RC replay worker."""

from __future__ import annotations

import time

from .gui_axis_normalization import clamp
from .models import RcReplaySample
from .replay_worker_seek import consume_replay_seek
from .replay_worker_state import ReplayWorkerState


def wait_for_replay_sample(owner, state: ReplayWorkerState, sample: RcReplaySample, *, rate: float) -> bool:
    """Wait until the sample is due. Return True when a seek interrupted the wait."""

    target_wall = state.start_wall + sample.time_s / rate
    while not owner._rc_replay_stop_event.is_set():
        if consume_replay_seek(owner, state, rate=rate):
            return True
        remaining = target_wall - time.monotonic()
        if remaining <= 0.0:
            break
        _maybe_update_wait_position(owner, state, rate=rate)
        time.sleep(min(remaining, 0.02))
    return False


def _maybe_update_wait_position(owner, state: ReplayWorkerState, *, rate: float) -> None:
    now = time.monotonic()
    if now - state.last_position_wall <= 0.15:
        return
    elapsed_s = clamp((now - state.start_wall) * rate, 0.0, state.local_duration_s)
    owner._set_rc_replay_position(elapsed_s)
    state.last_position_wall = now


__all__ = ["wait_for_replay_sample"]
