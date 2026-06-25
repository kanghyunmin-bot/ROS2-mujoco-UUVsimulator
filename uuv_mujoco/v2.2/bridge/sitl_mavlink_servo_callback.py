"""Shared SERVO_OUTPUT_RAW telemetry callback helper."""

from __future__ import annotations


def _call_servo_telemetry_callback(self, pwm_values: list[int], now_wall: float, *, label: str) -> None:
    if self._sitl_servo_telemetry_callback is None:
        return
    try:
        self._sitl_servo_telemetry_callback(pwm_values)
    except Exception as exc:
        if now_wall - self._sitl_json_servo_ignored_warn_wall > 3.0:
            print(f"[sitl_transport] {label} servo telemetry callback failed: {exc}", flush=True)
            self._sitl_json_servo_ignored_warn_wall = now_wall


__all__ = ["_call_servo_telemetry_callback"]
