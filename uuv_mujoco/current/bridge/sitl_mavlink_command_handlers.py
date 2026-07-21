"""Message handlers for the dedicated MAVLink command link."""

from __future__ import annotations

from sim.transport import SERVO_OUTPUT_RAW_TYPE, message_type, servo_output_raw_pwm_values

from .sitl_mavlink_servo_handlers import _call_servo_telemetry_callback


def _handle_command_link_message(self, msg, now_wall: float) -> None:
    msg_type = message_type(msg)
    if msg_type == "HEARTBEAT":
        _handle_command_heartbeat(self, msg, now_wall)
    elif msg_type == "COMMAND_ACK":
        self._handle_command_ack(msg, link_name="command")
    elif msg_type == SERVO_OUTPUT_RAW_TYPE:
        _handle_command_servo_output_raw(self, msg, now_wall)
    elif msg_type != "COMMAND_ACK":
        self._store_ap_mavlink_telemetry(msg, now_wall)


def _handle_command_heartbeat(self, msg, now_wall: float) -> None:
    self._store_ap_mavlink_telemetry(msg, now_wall)
    self._update_vehicle_heartbeat(msg, command_link=True)
    if not self._arm_mode_command_pending():
        self._request_command_servo_telemetry_stream(now_wall)
        self._request_command_ap_telemetry_stream(now_wall)


def _handle_command_servo_output_raw(self, msg, now_wall: float) -> None:
    self._store_ap_mavlink_telemetry(msg, now_wall)
    self._sitl_cmd_servo_last_msg_wall = now_wall
    self._sitl_cmd_servo_msg_count = int(getattr(self, "_sitl_cmd_servo_msg_count", 0)) + 1
    pwm_values = servo_output_raw_pwm_values(msg)
    if pwm_values is None:
        return
    _call_servo_telemetry_callback(self, pwm_values, now_wall, label="command-link")


__all__ = [
    "_handle_command_heartbeat",
    "_handle_command_link_message",
    "_handle_command_servo_output_raw",
]
