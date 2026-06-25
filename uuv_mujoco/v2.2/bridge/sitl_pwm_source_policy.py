"""PWM source ownership policy for SITL plant input."""

from __future__ import annotations


def is_external_pwm_source(source: str) -> bool:
    return str(source).startswith("replay_rcout")


def pwm_source_ignored_by_plant_replay(self, source: str, now_wall: float) -> bool:
    if not self._plant_replay_mode:
        return False
    if is_external_pwm_source(source):
        return False
    if now_wall - self._sitl_json_servo_ignored_warn_wall > 3.0:
        print(
            f"[sitl_transport] SITL({source}) servo packet ignored in plant_replay; "
            "recorded RCOUT/PWM is the authoritative plant input.",
            flush=True,
        )
        self._sitl_json_servo_ignored_warn_wall = now_wall
    return True


def pwm_source_blocked_by_external_override(self, source: str, now_wall: float) -> bool:
    if is_external_pwm_source(source):
        return False
    return (
        self._sitl_external_servo_override_until_wall > 0.0
        and now_wall < self._sitl_external_servo_override_until_wall
    )


__all__ = [
    "is_external_pwm_source",
    "pwm_source_blocked_by_external_override",
    "pwm_source_ignored_by_plant_replay",
]
