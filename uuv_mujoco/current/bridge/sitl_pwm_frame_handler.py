"""Top-level SITL PWM frame handling for MuJoCo plant input."""

from __future__ import annotations

from .sitl_pwm_activity import update_pwm_activity_state
from .sitl_pwm_output import dispatch_pwm_callback, log_pwm_debug_if_needed
from .sitl_pwm_safety import neutralize_all_min_pwm_if_needed, neutralize_disarmed_pwm_if_needed
from .sitl_pwm_source_policy import (
    is_external_pwm_source,
    pwm_source_blocked_by_external_override,
    pwm_source_ignored_by_plant_replay,
)


def _handle_pwm_values(self, pwm_values: list[int], now_wall: float, source: str) -> None:
    if pwm_source_ignored_by_plant_replay(self, source, now_wall):
        return
    if pwm_source_blocked_by_external_override(self, source, now_wall):
        return

    replay_owns_plant = bool(self._plant_replay_mode and is_external_pwm_source(source))
    pwm_values = neutralize_disarmed_pwm_if_needed(self, pwm_values, now_wall, source, replay_owns_plant)
    pwm_values = neutralize_all_min_pwm_if_needed(self, pwm_values, now_wall, source, replay_owns_plant)
    update_pwm_activity_state(self, pwm_values, now_wall, source)
    dispatch_pwm_callback(self, pwm_values)
    log_pwm_debug_if_needed(self, pwm_values, now_wall, source)


__all__ = ["_handle_pwm_values"]
