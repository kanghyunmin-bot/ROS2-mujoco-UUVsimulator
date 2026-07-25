"""MAVLink SERVO_OUTPUT_RAW drain loop for SitlTransport."""

from __future__ import annotations

from sim.transport import SERVO_OUTPUT_RAW_TYPE, message_type

from .qgc_mavlink_relay import relay_ap_message_to_qgc
from .sitl_mavlink_servo_handlers import _handle_servo_output_raw
from .sitl_mavlink_servo_receive import recv_servo_link_message


def drain_servo_mavlink(transport, now_wall: float) -> tuple[bool, list[int] | None]:
    got_any = False
    latest_pwm_values: list[int] | None = None
    for _ in range(transport._sitl_mavlink_poll_budget):
        msg = recv_servo_link_message(transport)
        if msg is None:
            break
        relay_ap_message_to_qgc(transport, msg)
        mtype = message_type(msg)
        if mtype == "HEARTBEAT":
            transport._handle_servo_link_heartbeat(msg, now_wall)
        elif mtype == "COMMAND_ACK":
            transport._handle_command_ack(msg, link_name="servo")
        elif mtype != SERVO_OUTPUT_RAW_TYPE:
            transport._store_ap_mavlink_telemetry(msg, now_wall)
        else:
            got_any = True
            pwm_values = _handle_servo_output_raw(transport, msg, now_wall)
            if pwm_values is not None and not transport._sitl_json_servo_fallback:
                latest_pwm_values = pwm_values
    return got_any, latest_pwm_values


__all__ = ["drain_servo_mavlink"]
