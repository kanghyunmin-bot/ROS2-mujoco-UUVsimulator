"""Callback and debug output helpers for SITL PWM frames."""

from __future__ import annotations


def dispatch_pwm_callback(self, pwm_values: list[int]) -> None:
    if self._sitl_servo_callback is None:
        return
    try:
        self._sitl_servo_callback(pwm_values)
    except Exception as exc:
        print(f"[sitl_transport] SITL servo callback failed: {exc}", flush=True)


def log_pwm_debug_if_needed(self, pwm_values: list[int], now_wall: float, source: str) -> None:
    if not self._sitl_cmd_debug:
        return
    pkt8 = tuple(int(v) for v in pwm_values[:8])
    if (self._sitl_last_servo_pkt != pkt8) and (now_wall - self._sitl_last_cmd_log > 0.15):
        print(f"[sitl_transport] SITL({source}) servo pwm[1..8]={pkt8}", flush=True)
        self._sitl_last_cmd_log = now_wall
        self._sitl_last_servo_pkt = pkt8


__all__ = ["dispatch_pwm_callback", "log_pwm_debug_if_needed"]
