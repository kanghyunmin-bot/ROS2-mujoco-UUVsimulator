"""Publish and status updates for the GUI RC replay worker."""

from __future__ import annotations

import time

from .models import RcReplaySample
from .replay_format import format_replay_time
from .replay_worker_state import ReplayWorkerState


def publish_replay_sample(
    owner,
    state: ReplayWorkerState,
    sample: RcReplaySample,
    *,
    rate: float,
    total_count: int,
) -> bool:
    if not owner.node.publish_rc_channels(sample.channels):
        state.stopped = True
        owner._set_rc_replay_status("replay failed: rc override publisher unavailable")
        return False
    _maybe_update_sample_status(owner, state, sample, rate=rate, total_count=total_count)
    return True


def _maybe_update_sample_status(
    owner,
    state: ReplayWorkerState,
    sample: RcReplaySample,
    *,
    rate: float,
    total_count: int,
) -> None:
    now = time.monotonic()
    if now - state.last_position_wall > 0.15:
        owner._set_rc_replay_position(sample.time_s)
        state.last_position_wall = now
    if now - state.last_status_wall > 0.5:
        state.last_status_wall = now
        owner._set_rc_replay_status(
            f"replay: {state.idx + 1}/{total_count}  {format_replay_time(sample.time_s)}  rate={rate:g}x"
        )


__all__ = ["publish_replay_sample"]
