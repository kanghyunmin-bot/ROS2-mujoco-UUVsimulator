"""Phase helpers for one GUI RC replay worker step."""

from __future__ import annotations

from .models import RcReplaySample
from .replay_worker_pause import handle_replay_pause
from .replay_worker_publish import publish_replay_sample
from .replay_worker_seek import consume_replay_seek
from .replay_worker_state import ReplayWorkerState


def stop_requested(owner, state: ReplayWorkerState) -> bool:
    if not owner._rc_replay_stop_event.is_set():
        return False
    state.stopped = True
    return True


def handle_seek_or_pause(owner, state: ReplayWorkerState, *, rate: float) -> bool:
    if consume_replay_seek(owner, state, rate=rate):
        return True
    handle_replay_pause(owner, state, rate=rate)
    return False


def replay_finished(samples: list[RcReplaySample], state: ReplayWorkerState) -> bool:
    return state.idx >= len(samples)


def current_replay_sample(samples: list[RcReplaySample], state: ReplayWorkerState) -> RcReplaySample:
    return samples[state.idx]


def publish_sample_and_advance(
    owner,
    samples: list[RcReplaySample],
    state: ReplayWorkerState,
    sample: RcReplaySample,
    *,
    rate: float,
) -> bool:
    if not publish_replay_sample(owner, state, sample, rate=rate, total_count=len(samples)):
        return False
    state.idx += 1
    return True


__all__ = [
    "current_replay_sample",
    "handle_seek_or_pause",
    "publish_sample_and_advance",
    "replay_finished",
    "stop_requested",
]
