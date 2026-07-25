"""Vehicle state and status-text callbacks for UuvGuiNode."""

from __future__ import annotations

import time


_SEVERITY_NAMES = {
    0: "EMERGENCY",
    1: "ALERT",
    2: "CRITICAL",
    3: "ERROR",
    4: "WARNING",
    5: "NOTICE",
    6: "INFO",
    7: "DEBUG",
}


def _severity_name(level: int) -> str:
    return _SEVERITY_NAMES.get(level, f"S{level}")


def _on_state(self, msg: State) -> None:
    self._touch("state")
    now = time.monotonic()
    with self._lock:
        was_connected = bool(self._snapshot.connected)
        self._snapshot.connected = bool(msg.connected)
        self._snapshot.armed = bool(msg.armed)
        self._snapshot.guided = bool(msg.guided)
        self._snapshot.manual_input = bool(msg.manual_input)
        self._snapshot.mode = msg.mode
        self._snapshot.system_status = int(msg.system_status)
        if bool(msg.connected) and not was_connected:
            self._vehicle_connected_since_wall = now
        elif not bool(msg.connected):
            self._vehicle_connected_since_wall = -1.0

    if msg.mode != self._last_mode_seen:
        self._push_event(f"mode -> {msg.mode}")
        self._last_mode_seen = msg.mode
    if self._last_armed_seen is None or bool(msg.armed) != self._last_armed_seen:
        self._push_event(f"armed -> {msg.armed}")
        self._last_armed_seen = bool(msg.armed)
    self._try_release_initial_depth_hold()


def _on_status_text(self, msg: StatusText) -> None:
    self._push_event(f"{_severity_name(int(msg.severity))}: {msg.text}")


__all__ = ["_on_state", "_on_status_text"]
