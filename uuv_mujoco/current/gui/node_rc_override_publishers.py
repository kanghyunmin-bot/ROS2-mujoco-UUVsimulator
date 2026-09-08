"""RC override publishers for UuvGuiNode."""

from __future__ import annotations

from typing import Iterable

from .config import (
    ALT_HOLD_RC_HEAVE_INVERT,
    PRIMARY_RC_CHANNEL_COUNT,
    RC_HEAVE_CHANNEL_INDEX,
    RC_NEUTRAL_PWM,
)
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
        if bool(getattr(self, "_arm_rc3_low_active", False)):
            return
        _advance_rc_override_generation(self)
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
    _cancel_prearm_for_external_release(self, "RC release requested")
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return
        _advance_rc_override_generation(self)
        _publish_rc_release_locked(self)


def publish_rc_neutral_then_release(self) -> None:
    """Publish zero pilot input, then release RC ownership after a short delay."""
    _cancel_prearm_for_external_release(self, "neutral/release requested")
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return
        generation = _advance_rc_override_generation(self)
        neutral = _build_rc_override_message(
            self,
            yaw=0.0,
            heave=0.0,
            forward=0.0,
            lateral=0.0,
        )
        for _ in range(self._rc_override_burst_count):
            self._rc_override_pub.publish(neutral)

    delay_s = float(getattr(self, "_rc_override_release_delay_s", 0.15))
    self._schedule_once(
        delay_s,
        lambda: _publish_rc_release_if_current(self, generation),
    )


def publish_rc_channels(self, channels: Iterable[int]) -> bool:
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return False
        if bool(getattr(self, "_arm_rc3_low_active", False)):
            return False
        _advance_rc_override_generation(self)
        msg = OverrideRCIn()
        msg.channels = sanitize_primary_rc_override_channels(channels)
        for _ in range(self._rc_override_burst_count):
            self._rc_override_pub.publish(msg)
        return True


def publish_rc_arm_low(self) -> bool:
    """Publish primary-axis neutral with only RC3 at ArduSub arm-low PWM."""

    return _publish_arm_rc3_frame(self, 1100)


def publish_rc_arm_neutral(self) -> bool:
    """Publish the post-arm/disarm primary-axis neutral RC frame."""

    return _publish_arm_rc3_frame(self, RC_NEUTRAL_PWM)


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


def _publish_arm_rc3_frame(self, rc3_pwm: int) -> bool:
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return False
        _advance_rc_override_generation(self)
        primary = [RC_NEUTRAL_PWM] * PRIMARY_RC_CHANNEL_COUNT
        primary[RC_HEAVE_CHANNEL_INDEX] = int(rc3_pwm)
        msg = OverrideRCIn()
        msg.channels = sanitize_primary_rc_override_channels(primary)
        for _ in range(self._rc_override_burst_count):
            self._rc_override_pub.publish(msg)
        return True


def _cancel_prearm_for_external_release(self, reason: str) -> None:
    sequence_lock = getattr(self, "_arm_rc_sequence_lock", None)
    if sequence_lock is None:
        return
    with sequence_lock:
        if not (
            bool(getattr(self, "_arm_rc3_low_active", False))
            or bool(getattr(self, "_arm_command_pending", False))
        ):
            return
        # Local import avoids a module import cycle: the sequence helper uses
        # the two dedicated arm-frame publishers above.
        from .node_arm_rc_sequence import (
            cancel_arm_rc_sequence,
            finish_arm_command_generation,
        )

        command_generation = int(self._arm_command_generation)
        cancel_arm_rc_sequence(self, reason, force_neutral=True)
        finish_arm_command_generation(self, command_generation)
        self._latest_arm_target = None


def _advance_rc_override_generation(self) -> int:
    generation = int(getattr(self, "_rc_override_publish_generation", 0)) + 1
    self._rc_override_publish_generation = generation
    return generation


def _publish_rc_release_locked(self) -> None:
    msg = make_rc_release_message()
    for _ in range(self._rc_override_burst_count):
        self._rc_override_pub.publish(msg)


def _publish_rc_release_if_current(self, generation: int) -> None:
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is None:
            return
        if int(getattr(self, "_rc_override_publish_generation", 0)) != int(generation):
            return
        _advance_rc_override_generation(self)
        _publish_rc_release_locked(self)


__all__ = [
    "publish_rc_arm_low",
    "publish_rc_arm_neutral",
    "publish_rc_channels",
    "publish_rc_neutral_then_release",
    "publish_rc_override",
    "publish_rc_release",
]
