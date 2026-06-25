"""Latest-frame forwarding cache for MAVROS RC override input."""

from __future__ import annotations

import time
from collections.abc import Sequence

from .ros2_rc_override_forwarding import forward_rc_override_to_sitl
from .ros2_rc_override_frame import rc_override_forward_frame
from .ros2_rc_override_warning import _warn_rc_override_not_forwarded


def store_pending_rc_override(self, channels: Sequence[int]) -> None:
    frame = rc_override_forward_frame(channels)
    now = time.monotonic()
    with self._mavros_rc_override_cache_lock:
        self._mavros_pending_rc_override_channels = frame
        self._mavros_pending_rc_override_wall = now


def service_pending_rc_override_forward(self) -> None:
    channels = _pending_frame_due(self, time.monotonic())
    if channels is None:
        return
    if forward_rc_override_to_sitl(self, channels):
        self._mavros_last_forwarded_rc_override_wall = time.monotonic()
    else:
        _warn_rc_override_not_forwarded(self)


def _pending_frame_due(self, now: float) -> list[int] | None:
    with self._mavros_rc_override_cache_lock:
        channels = self._mavros_pending_rc_override_channels
        pending_wall = float(self._mavros_pending_rc_override_wall)
        last_forward_wall = float(self._mavros_last_forwarded_rc_override_wall)
    if channels is None or pending_wall <= 0.0:
        return None
    if now - pending_wall > float(self._mavros_rc_override_stale_s):
        return None
    if last_forward_wall > 0.0 and now - last_forward_wall < float(self._mavros_rc_override_forward_period_s):
        return None
    return list(channels)


__all__ = ["service_pending_rc_override_forward", "store_pending_rc_override"]
