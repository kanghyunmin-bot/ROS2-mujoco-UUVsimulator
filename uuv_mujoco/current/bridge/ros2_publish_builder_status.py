"""SITL status ROS message builders for publish jobs."""

from __future__ import annotations

from .ros2_publish_state import RosPublishState
from .ros2_standard_messages import build_json_string_msg


def build_status_publish_builders(self, stamp, state: RosPublishState) -> dict[str, object]:
    sitl_sensor_replay_status_msg = None
    sitl_mavlink_telemetry_status_msg = None

    def get_sitl_sensor_replay_status_msg():
        nonlocal sitl_sensor_replay_status_msg
        if sitl_sensor_replay_status_msg is None:
            status = {"active": False}
            if self._sitl_transport is not None:
                getter = getattr(self._sitl_transport, "sensor_replay_status", None)
                if callable(getter):
                    status = dict(getter())
            status["sim_time_s"] = float(state.sim_t)
            sitl_sensor_replay_status_msg = build_json_string_msg(self.String, status)
        return sitl_sensor_replay_status_msg

    def get_sitl_mavlink_telemetry_status_msg():
        nonlocal sitl_mavlink_telemetry_status_msg
        if sitl_mavlink_telemetry_status_msg is None:
            status = {"active": False}
            if self._sitl_transport is not None:
                getter = getattr(self._sitl_transport, "mavlink_telemetry_status", None)
                if callable(getter):
                    status = dict(getter())
            status["sim_time_s"] = float(state.sim_t)
            sitl_mavlink_telemetry_status_msg = build_json_string_msg(self.String, status)
        return sitl_mavlink_telemetry_status_msg

    return {
        "sitl_sensor_replay_status": get_sitl_sensor_replay_status_msg,
        "sitl_mavlink_telemetry_status": get_sitl_mavlink_telemetry_status_msg,
    }
