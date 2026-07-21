"""Safety neutralization policy for SITL PWM frames."""

from __future__ import annotations

from sim.transport import all_active_outputs_at_min, has_nonneutral_pwm, neutral_pwm_frame


def neutralize_disarmed_pwm_if_needed(self, pwm_values: list[int], now_wall: float, source: str, replay_owns_plant: bool):
    if self._sitl_vehicle_armed or replay_owns_plant:
        return pwm_values
    nonneutral_disarmed = has_nonneutral_pwm(pwm_values)
    if nonneutral_disarmed and now_wall - self._sitl_last_disarmed_servo_warn_wall > 3.0:
        print(
            f"[sitl_transport] SITL({source}) servo output ignored while disarmed "
            f"pwm[1..8]={tuple(int(v) for v in pwm_values[:8])}",
            flush=True,
        )
        self._sitl_last_disarmed_servo_warn_wall = now_wall
    return neutral_pwm_frame(len(pwm_values))


def neutralize_all_min_pwm_if_needed(self, pwm_values: list[int], now_wall: float, source: str, replay_owns_plant: bool):
    if replay_owns_plant:
        return pwm_values
    if not all_active_outputs_at_min(pwm_values):
        return pwm_values
    if now_wall - self._sitl_last_all_min_servo_warn_wall > 3.0:
        print(
            f"[sitl_transport] SITL({source}) all-min motor frame treated as neutral "
            f"pwm[1..8]={tuple(int(v) for v in pwm_values[:8])}",
            flush=True,
        )
        self._sitl_last_all_min_servo_warn_wall = now_wall
    return neutral_pwm_frame(len(pwm_values))


__all__ = ["neutralize_all_min_pwm_if_needed", "neutralize_disarmed_pwm_if_needed"]
