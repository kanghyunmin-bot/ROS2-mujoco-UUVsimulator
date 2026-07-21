"""Clock and payload timestamp policy for controller-parity sensor replay."""

from __future__ import annotations

from .sitl_sensor_replay_payload_time import sensor_replay_payload_timestamp_for_sim_t
from .sitl_sensor_replay_servo_clock import sensor_replay_servo_frame_clock_time_s


def _sensor_replay_clock_time_s(self, sim_t: float) -> float | None:
    if self._sensor_replay_clock != "servo_frame":
        return float(sim_t)
    return sensor_replay_servo_frame_clock_time_s(self)


def _sensor_replay_payload_timestamp_for_sim_t(self, sim_t: float) -> float | None:
    return sensor_replay_payload_timestamp_for_sim_t(self, sim_t)


__all__ = ["_sensor_replay_clock_time_s", "_sensor_replay_payload_timestamp_for_sim_t"]
