"""Initial-depth release command helpers for the GUI node."""

from __future__ import annotations

from .node_initial_depth_state import (
    finish_initial_depth_release,
    initial_depth_release_can_start,
    mark_initial_depth_release_not_started,
    mark_initial_depth_release_started,
    queue_initial_depth_release,
    real_start_release_already_done,
)


def _vehicle_ready_for_initial_depth_release(self) -> bool:
    with self._lock:
        armed = bool(self._snapshot.armed)
        mode = str(self._snapshot.mode).upper()
    if not armed:
        return False
    if self._initial_depth_release_reason == "ALT_HOLD":
        return mode == "ALT_HOLD"
    return True


def _try_release_initial_depth_hold(self) -> None:
    if not initial_depth_release_can_start(self):
        return
    mark_initial_depth_release_started(self)

    def _on_release_done(ok: bool) -> None:
        finish_initial_depth_release(self, ok)

    started = self._call_trigger_service(
        self._initial_depth_release_client,
        "initial depth release",
        on_done=_on_release_done,
    )
    if not started:
        mark_initial_depth_release_not_started(self)


def _request_initial_depth_release_when_armed(self, reason: str) -> None:
    if not self._initial_depth_hold_opt_in:
        return
    if real_start_release_already_done(self):
        return
    queue_initial_depth_release(self, reason)
    self._try_release_initial_depth_hold()


def request_initial_depth_release_when_armed(self, reason: str) -> None:
    self._request_initial_depth_release_when_armed(reason)


__all__ = [
    "_request_initial_depth_release_when_armed",
    "_try_release_initial_depth_hold",
    "_vehicle_ready_for_initial_depth_release",
    "request_initial_depth_release_when_armed",
]
