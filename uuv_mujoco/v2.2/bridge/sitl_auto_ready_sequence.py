"""Auto arm/mode readiness sequence orchestration for SITL."""

from __future__ import annotations

from .sitl_auto_ready_actions import (
    mark_auto_ready_finished,
    request_auto_ready_arm_if_needed,
    request_auto_ready_mode_if_needed,
)
from .sitl_auto_ready_gates import (
    auto_ready_done_still_valid,
    auto_ready_is_disabled,
    auto_ready_target_mode,
    auto_ready_wait_state,
    mark_auto_ready_started_if_needed,
)


def _service_auto_ready_sequence(self, now_wall: float) -> None:
    if auto_ready_is_disabled(self):
        return
    target_mode = auto_ready_target_mode(self)
    if auto_ready_done_still_valid(self, target_mode):
        return
    wait_state = auto_ready_wait_state(self, now_wall)
    if wait_state is not None:
        self._set_auto_ready_state(wait_state, now_wall)
        return
    mark_auto_ready_started_if_needed(self, now_wall, target_mode)
    if request_auto_ready_arm_if_needed(self, now_wall):
        return
    if request_auto_ready_mode_if_needed(self, now_wall, target_mode):
        return
    mark_auto_ready_finished(self, now_wall)


__all__ = ["_service_auto_ready_sequence"]
