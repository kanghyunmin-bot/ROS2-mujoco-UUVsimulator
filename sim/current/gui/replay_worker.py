"""Playback worker loop for GUI RC replay."""

from __future__ import annotations

import time

from .models import RcReplaySample
from .replay_worker_finish import finish_replay_worker
from .replay_worker_state import ReplayWorkerState
from .replay_worker_step import advance_replay_worker


def _run_rc_replay(self, samples: list[RcReplaySample], rate: float, start_time_s: float) -> None:
    state = ReplayWorkerState(
        local_duration_s=samples[-1].time_s if samples else 0.0,
        idx=self._rc_replay_sample_index_for_time(start_time_s),
        start_wall=time.monotonic() - start_time_s / rate,
    )

    while state.idx < len(samples):
        if not advance_replay_worker(self, samples, state, rate=rate):
            break

    finish_replay_worker(self, state)
