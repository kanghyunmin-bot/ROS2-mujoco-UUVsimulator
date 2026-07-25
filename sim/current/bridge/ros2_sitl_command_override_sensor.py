"""Sensor-replay markers from internal SITL command override payloads."""

from __future__ import annotations

from .ros2_sitl_command_override_topic import COMMAND_OVERRIDE_TOPIC


def _payload_marks_sensor_replay(payload: dict[str, object]) -> bool:
    return "sensor_replay_start" in payload or "sensor_replay_rc_seen" in payload


def _mark_sensor_replay_input(self, payload: dict[str, object]) -> None:
    if not _payload_marks_sensor_replay(payload):
        return
    if self._sitl_transport is not None and hasattr(
        self._sitl_transport,
        "mark_sensor_replay_input_seen",
    ):
        with self._sitl_transport_lock:
            self._sitl_transport.mark_sensor_replay_input_seen(COMMAND_OVERRIDE_TOPIC)
    print(
        f"[bridge] {COMMAND_OVERRIDE_TOPIC} sensor_replay_start accepted",
        flush=True,
    )


__all__ = ["_mark_sensor_replay_input", "_payload_marks_sensor_replay"]
