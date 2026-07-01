"""Neutral/non-neutral activity tracking for SITL PWM frames."""

from __future__ import annotations

from sim.transport import active_pwm_values, has_nonneutral_pwm


def update_pwm_activity_state(self, pwm_values: list[int], now_wall: float, source: str) -> None:
    valid_pwm = active_pwm_values(pwm_values)
    nonneutral = has_nonneutral_pwm(pwm_values)
    if nonneutral:
        self._sitl_last_nonneutral_servo_wall = now_wall
        return
    since_nonneutral = (
        now_wall - self._sitl_last_nonneutral_servo_wall
        if self._sitl_last_nonneutral_servo_wall > 0.0
        else now_wall - self._sitl_first_servo_wall
    )
    if since_nonneutral <= 3.0 or now_wall - self._sitl_last_neutral_warn_wall <= 3.0:
        return
    if not getattr(self, "_sitl_cmd_debug", False):
        self._sitl_last_neutral_warn_wall = now_wall
        return
    if not valid_pwm:
        print(
            f"[sitl_transport] SITL({source}) servo stream has no active outputs (all 0/65535). "
            "Check: vehicle ARM state and JSON sensor stream health.",
            flush=True,
        )
    else:
        print(
            f"[sitl_transport] SITL({source}) servo stream is neutral (all near 1500). "
            "Check: vehicle ARM state, QGC joystick enabled, MANUAL mode.",
            flush=True,
        )
    self._sitl_last_neutral_warn_wall = now_wall


__all__ = ["update_pwm_activity_state"]
