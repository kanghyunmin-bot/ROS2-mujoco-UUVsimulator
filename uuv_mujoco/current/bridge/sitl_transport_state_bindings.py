"""State and replay method bindings for SitlTransport."""

from __future__ import annotations

from bridge import (
    sitl_sensor_replay_runtime,
    sitl_transport_model_state,
    sitl_transport_status_runtime,
)


class SitlTransportStateBindings:
    _sensor_slice = sitl_transport_model_state._sensor_slice

    _sensor_replay_clock_time_s = sitl_sensor_replay_runtime._sensor_replay_clock_time_s
    sensor_replay_status = sitl_sensor_replay_runtime.sensor_replay_status

    mavlink_telemetry_status = sitl_transport_status_runtime.mavlink_telemetry_status

    _remember_sensor_replay_frame = sitl_sensor_replay_runtime._remember_sensor_replay_frame
    _sensor_replay_payload_timestamp_for_sim_t = (
        sitl_sensor_replay_runtime._sensor_replay_payload_timestamp_for_sim_t
    )
    _sensor_replay_frame_at = sitl_sensor_replay_runtime._sensor_replay_frame_at

    pressure_abs_from_depth_m = sitl_transport_model_state.pressure_abs_from_depth_m
    base_pos_world = sitl_transport_model_state.base_pos_world
    estimate_base_velocity_enu = sitl_transport_model_state.estimate_base_velocity_enu
    sitl_rangefinder_from_model = sitl_transport_model_state.sitl_rangefinder_from_model
    estimate_vertical_state = sitl_transport_model_state.estimate_vertical_state

    vehicle_armed = sitl_transport_status_runtime.vehicle_armed
    vehicle_mode = sitl_transport_status_runtime.vehicle_mode
    last_rc_override_values = sitl_transport_status_runtime.last_rc_override_values
    last_rc_override_age_s = sitl_transport_status_runtime.last_rc_override_age_s

    mark_sensor_replay_input_seen = sitl_sensor_replay_runtime.mark_sensor_replay_input_seen


__all__ = ["SitlTransportStateBindings"]
