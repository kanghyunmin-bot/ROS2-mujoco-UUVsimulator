"""ArduSub 4.1.2 low-RC3 pre-arm sequencing for the GUI command path."""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any


ARM_LOW_RC3_PWM = 1100
ARM_NEUTRAL_RC3_PWM = 1500
DEFAULT_ARM_LOW_REPEAT_COUNT = 3
DEFAULT_ARM_LOW_SETTLE_S = 0.15


def initialize_arm_rc_sequence_state(owner: Any) -> None:
    """Initialize the generation-guarded pre-arm RC sequence state."""

    owner._arm_rc_sequence_lock = threading.RLock()
    owner._arm_rc_sequence_generation = 0
    owner._arm_command_generation = 0
    owner._arm_command_pending = False
    owner._arm_request_in_flight_generation = None
    owner._arm_rc3_low_active = False
    owner._arm_rc3_precondition_ready = False
    owner._arm_rc3_precondition_deadline = -1.0

    repeat_raw = getattr(
        owner,
        "_arm_rc3_low_repeat_count",
        os.environ.get(
            "UUV_GUI_ARM_LOW_RC3_REPEAT_COUNT",
            str(DEFAULT_ARM_LOW_REPEAT_COUNT),
        ),
    )
    try:
        repeat_count = int(float(repeat_raw))
    except (TypeError, ValueError):
        repeat_count = DEFAULT_ARM_LOW_REPEAT_COUNT
    owner._arm_rc3_low_repeat_count = max(2, min(10, repeat_count))

    settle_raw = getattr(
        owner,
        "_arm_rc3_low_settle_s",
        os.environ.get(
            "UUV_GUI_ARM_LOW_RC3_SETTLE_S",
            str(DEFAULT_ARM_LOW_SETTLE_S),
        ),
    )
    try:
        settle_s = float(settle_raw)
    except (TypeError, ValueError):
        settle_s = DEFAULT_ARM_LOW_SETTLE_S
    if not math.isfinite(settle_s):
        settle_s = DEFAULT_ARM_LOW_SETTLE_S
    owner._arm_rc3_low_settle_s = max(0.05, min(0.50, settle_s))


def prepare_arm_rc_sequence_for_target(owner: Any, target_value: bool) -> int:
    """Cancel an older sequence and establish a new command generation."""

    _ensure_initialized(owner)
    with owner._arm_rc_sequence_lock:
        was_low_active = bool(owner._arm_rc3_low_active)
        owner._arm_rc_sequence_generation += 1
        owner._arm_command_generation += 1
        command_generation = int(owner._arm_command_generation)
        owner._arm_request_in_flight_generation = None
        owner._arm_rc3_low_active = False
        owner._arm_rc3_precondition_ready = False
        owner._arm_rc3_precondition_deadline = -1.0
        should_restore = bool(was_low_active or not bool(target_value))
        restored = _publish_arm_neutral(owner) if should_restore else True

    if should_restore:
        owner._push_event(
            "arm RC sequence restored RC3=1500 "
            f"for {'replacement request' if target_value else 'disarm'}"
            f"{'' if restored else ' (publisher unavailable)'}"
        )
    return command_generation


def arm_command_request_current(
    owner: Any,
    target_value: bool,
    command_generation: int | None,
) -> bool:
    """Return whether a callback still belongs to the active GUI command."""

    _ensure_initialized(owner)
    latest_target = getattr(owner, "_latest_arm_target", None)
    if latest_target is not None and bool(target_value) != bool(latest_target):
        return False
    if command_generation is None:
        return True
    with owner._arm_rc_sequence_lock:
        return int(command_generation) == int(owner._arm_command_generation)


def current_arm_command_generation(owner: Any) -> int:
    """Return the active high-level arm/disarm command generation."""

    _ensure_initialized(owner)
    with owner._arm_rc_sequence_lock:
        return int(owner._arm_command_generation)


def finish_arm_command_generation(owner: Any, command_generation: int) -> bool:
    """Finish one command and invalidate all of its future/retry callbacks."""

    _ensure_initialized(owner)
    with owner._arm_rc_sequence_lock:
        if int(command_generation) != int(owner._arm_command_generation):
            return False
        owner._arm_command_pending = False
        owner._arm_request_in_flight = False
        owner._arm_request_in_flight_generation = None
        owner._arm_command_generation += 1
        return True


def ensure_arm_rc_precondition(
    owner: Any,
    *,
    deadline: float,
    attempt: int,
    command_generation: int,
) -> bool:
    """Start or await the low-RC3 precondition.

    Returns ``True`` while the arm request must remain deferred and ``False``
    once the current generation has completed its short low-RC3 settle.
    """

    _ensure_initialized(owner)
    with owner._arm_rc_sequence_lock:
        if int(command_generation) != int(owner._arm_command_generation):
            return True
        if bool(owner._arm_rc3_precondition_ready):
            return False
        if bool(owner._arm_rc3_low_active):
            return True
        generation = int(owner._arm_rc_sequence_generation)
        owner._arm_rc3_low_active = True
        owner._arm_rc3_precondition_deadline = float(deadline)
        repeat_count = int(owner._arm_rc3_low_repeat_count)
        settle_s = float(owner._arm_rc3_low_settle_s)
        # Keep the sequence lock through the RC publish.  The state callback
        # follows the same sequence->publisher lock order, so an armed-state
        # neutral frame can never be followed by this generation's stale low
        # pulse.
        low_published = _publish_arm_low(owner)

    if not low_published:
        cancel_arm_rc_sequence(
            owner,
            "RC override publisher unavailable",
            expected_generation=generation,
            force_neutral=True,
        )
        owner._push_event(
            "arm blocked: RC override publisher is required for the "
            "ArduSub 4.1.2 low-RC3 precondition"
        )
        finish_arm_command_generation(owner, command_generation)
        return True

    owner._push_event(
        "arm precondition: primary axes neutral, RC3=1100 "
        f"({repeat_count} pulses over {settle_s:.2f}s)"
    )
    pulse_interval_s = settle_s / float(repeat_count)
    for pulse_index in range(1, repeat_count):
        owner._schedule_once(
            pulse_interval_s * pulse_index,
            lambda generation=generation: _publish_low_if_current(
                owner,
                generation,
            ),
        )
    owner._schedule_once(
        settle_s,
        lambda: _complete_precondition_if_current(
            owner,
            generation=generation,
            deadline=float(deadline),
            attempt=int(attempt),
            command_generation=int(command_generation),
        ),
    )
    owner._schedule_once(
        max(0.0, float(deadline) - time.monotonic()),
        lambda: _timeout_if_current(
            owner,
            generation,
            int(command_generation),
        ),
    )
    return True


def cancel_arm_rc_sequence(
    owner: Any,
    reason: str,
    *,
    expected_generation: int | None = None,
    force_neutral: bool = False,
) -> bool:
    """Invalidate scheduled low pulses and restore the RC3=1500 frame."""

    _ensure_initialized(owner)
    with owner._arm_rc_sequence_lock:
        generation = int(owner._arm_rc_sequence_generation)
        if expected_generation is not None and generation != int(expected_generation):
            return False
        was_low_active = bool(owner._arm_rc3_low_active)
        if not was_low_active and not force_neutral:
            return False
        owner._arm_rc_sequence_generation = generation + 1
        owner._arm_rc3_low_active = False
        owner._arm_rc3_precondition_ready = False
        owner._arm_rc3_precondition_deadline = -1.0
        # Serialize the restore with every generation-checked low pulse.  Lock
        # order is always arm-sequence then RC-publisher.
        published = _publish_arm_neutral(owner)

    owner._push_event(
        "arm RC sequence restored RC3=1500: "
        f"{reason}{'' if published else ' (publisher unavailable)'}"
    )
    return published


def arm_rc_sequence_failed(owner: Any, reason: str) -> None:
    """Restore neutral after a failed arm transport attempt."""

    cancel_arm_rc_sequence(owner, reason, force_neutral=True)


def on_arm_state_observed(owner: Any, armed: bool) -> None:
    """Consume `/mavros/state` and neutralize RC3 on arm confirmation."""

    _ensure_initialized(owner)
    with owner._arm_rc_sequence_lock:
        latest_target = getattr(owner, "_latest_arm_target", None)
        if (
            latest_target is not None
            and bool(latest_target) == bool(armed)
            and bool(owner._arm_command_pending)
        ):
            finish_arm_command_generation(
                owner,
                int(owner._arm_command_generation),
            )
        if bool(armed) and bool(owner._arm_rc3_low_active):
            cancel_arm_rc_sequence(
                owner,
                "armed state confirmed",
                force_neutral=True,
            )
            return
        if latest_target is False and bool(owner._arm_rc3_low_active):
            cancel_arm_rc_sequence(
                owner,
                "disarmed target observed",
                force_neutral=True,
            )


def arm_rc_low_active(owner: Any) -> bool:
    """Return whether ordinary pilot frames must be suppressed."""

    return bool(getattr(owner, "_arm_rc3_low_active", False))


def _publish_low_if_current(owner: Any, generation: int) -> None:
    with owner._arm_rc_sequence_lock:
        current = int(owner._arm_rc_sequence_generation) == int(generation)
        active = bool(owner._arm_rc3_low_active)
        target_is_arm = getattr(owner, "_latest_arm_target", None) is True
        deadline = float(owner._arm_rc3_precondition_deadline)
        command_generation = int(owner._arm_command_generation)
        if not current or not active or not target_is_arm:
            return
        if time.monotonic() >= deadline:
            timed_out = True
            published = True
        else:
            timed_out = False
            # See ensure_arm_rc_precondition(): the state observer cannot
            # restore neutral between this generation check and the publish.
            published = _publish_arm_low(owner)
    if timed_out:
        _timeout_if_current(owner, generation, command_generation)
        return
    if not published:
        cancel_arm_rc_sequence(
            owner,
            "RC override publisher disappeared during pre-arm settle",
            expected_generation=generation,
            force_neutral=True,
        )
        finish_arm_command_generation(owner, command_generation)


def _complete_precondition_if_current(
    owner: Any,
    *,
    generation: int,
    deadline: float,
    attempt: int,
    command_generation: int,
) -> None:
    with owner._arm_rc_sequence_lock:
        if int(owner._arm_rc_sequence_generation) != int(generation):
            return
        if int(owner._arm_command_generation) != int(command_generation):
            return
        if not bool(owner._arm_rc3_low_active):
            return
        if getattr(owner, "_latest_arm_target", None) is not True:
            return
        if time.monotonic() >= float(deadline):
            timed_out = True
        else:
            timed_out = False
            owner._arm_rc3_precondition_ready = True
    if timed_out:
        _timeout_if_current(owner, generation, command_generation)
        return
    owner._push_event("arm precondition settled; sending arming request")
    owner._send_arm_request(
        True,
        float(deadline),
        int(attempt),
        request_generation=int(command_generation),
    )


def _timeout_if_current(
    owner: Any,
    generation: int,
    command_generation: int,
) -> None:
    with owner._arm_rc_sequence_lock:
        if int(owner._arm_rc_sequence_generation) != int(generation):
            return
        if int(owner._arm_command_generation) != int(command_generation):
            return
        if not bool(owner._arm_rc3_low_active):
            return
        cancel_arm_rc_sequence(
            owner,
            "arm target timeout",
            expected_generation=generation,
            force_neutral=True,
        )
        finish_arm_command_generation(owner, command_generation)
    owner._push_event("arm target timeout: armed=True")


def publish_arm_ack_neutral(owner: Any, reason: str) -> bool:
    """Publish RC3 neutral after an accepted service ACK without releasing the guard."""

    _ensure_initialized(owner)
    with owner._arm_rc_sequence_lock:
        if not bool(owner._arm_rc3_low_active):
            return False
        published = _publish_arm_neutral(owner)
    owner._push_event(
        "arm RC sequence sent RC3=1500 while awaiting state: "
        f"{reason}{'' if published else ' (publisher unavailable)'}"
    )
    return published


def _publish_arm_low(owner: Any) -> bool:
    bound_publisher = getattr(owner, "publish_rc_arm_low", None)
    if callable(bound_publisher):
        return bool(bound_publisher())
    # Keep ROS message dependencies out of this policy module's import path.
    from .node_rc_override_publishers import publish_rc_arm_low

    return bool(publish_rc_arm_low(owner))


def _publish_arm_neutral(owner: Any) -> bool:
    bound_publisher = getattr(owner, "publish_rc_arm_neutral", None)
    if callable(bound_publisher):
        return bool(bound_publisher())
    # Keep ROS message dependencies out of this policy module's import path.
    from .node_rc_override_publishers import publish_rc_arm_neutral

    return bool(publish_rc_arm_neutral(owner))


def _ensure_initialized(owner: Any) -> None:
    if not hasattr(owner, "_arm_rc_sequence_lock"):
        initialize_arm_rc_sequence_state(owner)


__all__ = [
    "ARM_LOW_RC3_PWM",
    "ARM_NEUTRAL_RC3_PWM",
    "arm_command_request_current",
    "arm_rc_low_active",
    "arm_rc_sequence_failed",
    "cancel_arm_rc_sequence",
    "ensure_arm_rc_precondition",
    "finish_arm_command_generation",
    "initialize_arm_rc_sequence_state",
    "on_arm_state_observed",
    "prepare_arm_rc_sequence_for_target",
    "publish_arm_ack_neutral",
    "current_arm_command_generation",
]
