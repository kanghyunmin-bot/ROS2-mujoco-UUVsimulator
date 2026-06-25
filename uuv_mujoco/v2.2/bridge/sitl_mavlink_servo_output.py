"""SERVO_OUTPUT_RAW handling for SitlTransport."""

from __future__ import annotations

from sim.transport import servo_output_raw_pwm_values

from .sitl_mavlink_servo_callback import _call_servo_telemetry_callback


def _handle_servo_output_raw(self, msg, now_wall: float) -> list[int] | None:
    self._store_ap_mavlink_telemetry(msg, now_wall)
    self._sitl_client_last_wall = now_wall
    self._sitl_mav_last_msg_wall = now_wall
    self._sitl_servo_link_servo_last_msg_wall = now_wall
    self._sitl_servo_link_servo_msg_count = int(getattr(self, "_sitl_servo_link_servo_msg_count", 0)) + 1
    if self._sitl_first_servo_wall <= 0.0:
        self._sitl_first_servo_wall = now_wall
    self._sitl_last_command_stale_wall = -1.0
    pwm_values = servo_output_raw_pwm_values(msg)
    if pwm_values is None:
        return None
    if _servo_link_rcout_callback_allowed(self, now_wall):
        _call_servo_telemetry_callback(self, pwm_values, now_wall, label="SITL")
    return pwm_values


def _servo_link_rcout_callback_allowed(self, now_wall: float) -> bool:
    if not bool(getattr(self, "_sitl_command_link_telemetry_enabled", False)):
        return True
    last_command_servo = float(getattr(self, "_sitl_cmd_servo_last_msg_wall", -1.0))
    if last_command_servo <= 0.0:
        return True
    fresh_s = max(1.5, 2.5 / max(float(getattr(self, "_sitl_rcout_telemetry_hz", 2.0)), 0.5))
    return now_wall - last_command_servo > fresh_s


def _warn_waiting_for_servo_output(self, got_any: bool, now_wall: float) -> None:
    if got_any or self._sitl_json_servo_fallback:
        return
    no_msg_age = (
        now_wall - self._sitl_mav_last_msg_wall
        if self._sitl_mav_last_msg_wall > 0.0
        else float("inf")
    )
    if (
        no_msg_age >= self._sitl_mav_wait_warn_interval_s
        and now_wall - self._sitl_mav_last_wait_warn_wall >= self._sitl_mav_wait_warn_interval_s
    ):
        print(
            "[sitl_transport] Waiting for SITL MAVLink SERVO_OUTPUT_RAW "
            f"on {self._sitl_mavlink_endpoint}",
            flush=True,
        )
        self._sitl_mav_last_wait_warn_wall = now_wall


__all__ = ["_handle_servo_output_raw", "_servo_link_rcout_callback_allowed", "_warn_waiting_for_servo_output"]
