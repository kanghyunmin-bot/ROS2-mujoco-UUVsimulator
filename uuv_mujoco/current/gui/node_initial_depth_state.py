"""Initial-depth release state transitions for the GUI node."""

from __future__ import annotations


def initial_depth_release_can_start(self) -> bool:
    if not self._initial_depth_hold_opt_in:
        return False
    if not self._initial_depth_release_pending:
        return False
    if self._initial_depth_release_in_flight:
        return False
    return bool(self._vehicle_ready_for_initial_depth_release())


def mark_initial_depth_release_started(self) -> None:
    self._initial_depth_release_in_flight = True


def finish_initial_depth_release(self, ok: bool) -> None:
    self._initial_depth_release_in_flight = False
    self._initial_depth_release_pending = not bool(ok)
    if ok:
        self._initial_depth_release_reason = ""


def mark_initial_depth_release_not_started(self) -> None:
    self._initial_depth_release_in_flight = False
    self._initial_depth_release_pending = True


def real_start_release_already_done(self) -> bool:
    with self._lock:
        return bool(self._snapshot.real_start_required) and bool(self._snapshot.real_start_released)


def queue_initial_depth_release(self, reason: str) -> None:
    self._initial_depth_release_pending = True
    self._initial_depth_release_reason = str(reason).strip() or "unknown"
    self._push_event(f"initial depth hold release waiting for armed state ({reason})")


__all__ = [
    "finish_initial_depth_release",
    "initial_depth_release_can_start",
    "mark_initial_depth_release_not_started",
    "mark_initial_depth_release_started",
    "queue_initial_depth_release",
    "real_start_release_already_done",
]
