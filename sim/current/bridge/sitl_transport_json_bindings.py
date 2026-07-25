"""JSON servo and sensor method bindings for SitlTransport."""

from __future__ import annotations

from bridge import sitl_json_sensor_runtime, sitl_json_servo_runtime


class SitlTransportJsonBindings:
    _poll_servo_endpoint = sitl_json_servo_runtime._poll_servo_endpoint
    _service_plant_replay_timeout = sitl_json_servo_runtime._service_plant_replay_timeout
    poll_servo = sitl_json_servo_runtime.poll_servo

    _send_sitl_json_payload = sitl_json_sensor_runtime._send_sitl_json_payload
    _payload_from_state = sitl_json_sensor_runtime._payload_from_state
    _send_immediate_sensor_replay_reply = sitl_json_sensor_runtime._send_immediate_sensor_replay_reply
    send_state = sitl_json_sensor_runtime.send_state


__all__ = ["SitlTransportJsonBindings"]
