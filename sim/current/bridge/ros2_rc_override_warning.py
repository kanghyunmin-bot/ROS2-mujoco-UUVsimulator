"""Warning throttles for MAVROS RC override input."""

from __future__ import annotations

import time


def _warn_rc_override_not_forwarded(self) -> None:
    now = time.monotonic()
    if now - self._mavros_last_rc_override_warn_wall > 2.0:
        print(
            "[ros2] /mavros/rc/override received but not mirrored to /mavros/rc/in "
            "because MAVLink forwarding is not ready",
            flush=True,
        )
        self._mavros_last_rc_override_warn_wall = now


__all__ = ["_warn_rc_override_not_forwarded"]
