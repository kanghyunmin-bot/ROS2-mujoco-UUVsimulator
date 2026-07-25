"""Single-step orchestration for the GUI RC replay worker."""

from __future__ import annotations

from .models import RcReplaySample
from .replay_worker_step_phases import (
    current_replay_sample,
    handle_seek_or_pause,
    publish_sample_and_advance,
    replay_finished,
    stop_requested,
)
from .replay_worker_state import ReplayWorkerState
from .replay_worker_wait import wait_for_replay_sample


def advance_replay_worker(
    owner,
    samples: list[RcReplaySample],
    state: ReplayWorkerState,
    *,
    rate: float,
) -> bool:
    if stop_requested(owner, state):
        return False
    if handle_seek_or_pause(owner, state, rate=rate):
        return True
    if stop_requested(owner, state) or replay_finished(samples, state):
        return False

    sample = current_replay_sample(samples, state)
    if wait_for_replay_sample(owner, state, sample, rate=rate):
        return True
    if replay_finished(samples, state) or stop_requested(owner, state):
        return False

    return publish_sample_and_advance(owner, samples, state, sample, rate=rate)


__all__ = ["advance_replay_worker"]
