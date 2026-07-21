"""Plant replay timeout handling for external RCOUT override input."""

from __future__ import annotations


def _service_plant_replay_timeout(self, now_wall: float) -> None:
    if not self._plant_replay_mode:
        return
    until_wall = float(self._sitl_external_servo_override_until_wall)
    if until_wall <= 0.0 or now_wall < until_wall:
        return
    self._sitl_external_servo_override_until_wall = -1.0
    self._handle_pwm_values([1500] * 8, now_wall, source="replay_rcout_timeout")


__all__ = ["_service_plant_replay_timeout"]
