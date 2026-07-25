"""Finalization for the GUI RC replay worker."""

from __future__ import annotations

from .replay_worker_state import ReplayWorkerState


def finish_replay_worker(owner, state: ReplayWorkerState) -> None:
    owner.node.publish_rc_release()
    owner._rc_replay_stop_event.clear()
    owner._rc_replay_pause_event.clear()
    owner._set_rc_replay_pause_button("Pause")
    if state.stopped:
        owner._set_rc_replay_status("replay: stopped")
    else:
        owner._set_rc_replay_position(state.local_duration_s, force=True)
        owner._set_rc_replay_status("replay: finished")


__all__ = ["finish_replay_worker"]
