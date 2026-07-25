"""RC override publishers for UuvGuiNode."""

from __future__ import annotations

from typing import Iterable

from .config import ALT_HOLD_RC_HEAVE_INVERT
from .helpers import (
    make_rc_override_message,
    make_rc_release_message,
    sanitize_primary_rc_override_channels,
)
from .runtime import OverrideRCIn


def publish_rc_override(
    self,
    *,
    yaw: float,
    heave: float,
    forward: float,
    lateral: float,
    pitch: float = 0.0,
    roll: float = 0.0,
) -> None:
    del pitch
    del roll
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return
        msg = _build_rc_override_message(
            self,
            yaw=yaw,
            heave=heave,
            forward=forward,
            lateral=lateral,
        )
        for _ in range(self._rc_override_burst_count):
            self._rc_override_pub.publish(msg)


def publish_rc_release(self) -> None:
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return
        msg = make_rc_release_message()
        for _ in range(self._rc_override_burst_count):
            self._rc_override_pub.publish(msg)


def publish_rc_channels(self, channels: Iterable[int]) -> bool:
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return False
        msg = OverrideRCIn()
        msg.channels = sanitize_primary_rc_override_channels(channels)
        for _ in range(self._rc_override_burst_count):
            self._rc_override_pub.publish(msg)
        return True


def _build_rc_override_message(
    self,
    *,
    yaw: float,
    heave: float,
    forward: float,
    lateral: float,
) -> OverrideRCIn:
    return make_rc_override_message(
        self._active_layout(),
        yaw=yaw,
        heave=heave,
        forward=forward,
        lateral=lateral,
        invert_heave=_should_invert_heave(self),
    )


def _should_invert_heave(self) -> bool:
    with self._lock:
        mode = str(self._snapshot.mode).upper()
    return bool(ALT_HOLD_RC_HEAVE_INVERT and mode == "ALT_HOLD")


__all__ = ["publish_rc_override", "publish_rc_release", "publish_rc_channels"]
