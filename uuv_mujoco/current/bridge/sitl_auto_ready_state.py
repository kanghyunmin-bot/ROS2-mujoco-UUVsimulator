"""Auto-ready state and log throttling helpers."""

from __future__ import annotations


def _set_auto_ready_state(self, state: str, now_wall: float, *, detail: str = "") -> None:
    state = str(state or "unknown")
    should_log = state != self._sitl_auto_ready_state or now_wall - self._sitl_auto_ready_last_log_wall >= 3.0
    self._sitl_auto_ready_state = state
    if not should_log:
        return
    self._sitl_auto_ready_last_log_wall = now_wall
    suffix = f": {detail}" if detail else ""
    print(f"[sitl_transport] auto-ready {state}{suffix}", flush=True)


__all__ = ["_set_auto_ready_state"]
