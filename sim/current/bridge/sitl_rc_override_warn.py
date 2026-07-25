"""Warning throttles for RC override forwarding."""

from __future__ import annotations

import time


def _warn_rc_override_not_forwarded(self, reason: str) -> None:
    now = time.monotonic()
    if now - self._sitl_last_rc_override_warn_wall < 2.0:
        return
    print(f"[sitl_transport] RC override not forwarded to ArduSub: {reason}", flush=True)
    self._sitl_last_rc_override_warn_wall = now


__all__ = ["_warn_rc_override_not_forwarded"]
