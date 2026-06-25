"""Method binding surface for UuvGuiNode."""

from __future__ import annotations

from . import node_commanding
from . import node_state_runtime
from . import node_telemetry_callbacks
from . import node_vehicle_info


def bind_uuv_gui_node_methods(cls: type) -> None:
    bind_state_runtime(cls)
    bind_vehicle_info(cls)
    bind_commanding(cls)
    bind_telemetry_callbacks(cls)


def bind_state_runtime(cls: type) -> None:
    cls._safe_count_publishers = node_state_runtime.safe_count_publishers
    cls._safe_count_subscribers = node_state_runtime.safe_count_subscribers
    cls._service_ready = staticmethod(node_state_runtime.service_ready)
    cls._payload_float = staticmethod(node_state_runtime.payload_float)
    cls._effective_backend = node_state_runtime.effective_backend
    cls._active_layout = node_state_runtime.active_layout
    cls.backend_label = node_state_runtime.backend_label
    cls.rc_mapping_summary = node_state_runtime.rc_mapping_summary
    cls.control_readiness = node_state_runtime.control_readiness
    cls._probe_backend = node_state_runtime.probe_backend
    cls._touch = node_state_runtime.touch
    cls._push_event = node_state_runtime.push_event
    cls.snapshot = node_state_runtime.snapshot
    cls._sitl_mavlink_command_alive = node_state_runtime.command_alive
    cls._sitl_extnav_ready = node_state_runtime.extnav_ready


def bind_vehicle_info(cls: type) -> None:
    cls.request_vehicle_info = node_vehicle_info.request_vehicle_info
    cls._on_vehicle_info_response = node_vehicle_info._on_vehicle_info_response


def bind_commanding(cls: type) -> None:
    cls._call_trigger_service = node_commanding._call_trigger_service
    cls._vehicle_ready_for_initial_depth_release = node_commanding._vehicle_ready_for_initial_depth_release
    cls._try_release_initial_depth_hold = node_commanding._try_release_initial_depth_hold
    cls._request_initial_depth_release_when_armed = node_commanding._request_initial_depth_release_when_armed
    cls.request_initial_depth_release_when_armed = node_commanding.request_initial_depth_release_when_armed
    cls._schedule_once = node_commanding._schedule_once
    cls._state_age_s = node_commanding._state_age_s
    cls._fresh_vehicle_state = node_commanding._fresh_vehicle_state
    cls._arm_mode_settle_left_s = node_commanding._arm_mode_settle_left_s
    cls._arm_mode_gate_reason = node_commanding._arm_mode_gate_reason
    cls._arm_target_reached = node_commanding._arm_target_reached
    cls._mode_target_reached = node_commanding._mode_target_reached
    cls._retry_arm_request = node_commanding._retry_arm_request
    cls._retry_mode_request = node_commanding._retry_mode_request
    cls._publish_command_override = node_commanding._publish_command_override
    cls._send_arm_request = node_commanding._send_arm_request
    cls.arm = node_commanding.arm
    cls._on_arm_response = node_commanding._on_arm_response
    cls.set_mode = node_commanding.set_mode
    cls._send_mode_request = node_commanding._send_mode_request
    cls._on_mode_response = node_commanding._on_mode_response
    cls.publish_rc_override = node_commanding.publish_rc_override
    cls.publish_manual_control = node_commanding.publish_manual_control
    cls.publish_rc_release = node_commanding.publish_rc_release
    cls.publish_rc_channels = node_commanding.publish_rc_channels
    cls.publish_ping360_config = node_commanding.publish_ping360_config
    cls.publish_ping360_enabled = node_commanding.publish_ping360_enabled


def bind_telemetry_callbacks(cls: type) -> None:
    cls._on_state = node_telemetry_callbacks._on_state
    cls._on_imu = node_telemetry_callbacks._on_imu
    cls._on_battery = node_telemetry_callbacks._on_battery
    cls._on_pose = node_telemetry_callbacks._on_pose
    cls._on_odom = node_telemetry_callbacks._on_odom
    cls._on_local_odom = node_telemetry_callbacks._on_local_odom
    cls._on_rovio_odom = node_telemetry_callbacks._on_rovio_odom
    cls._on_dvl_odom = node_telemetry_callbacks._on_dvl_odom
    cls._on_velocity = node_telemetry_callbacks._on_velocity
    cls._on_velocity_body = node_telemetry_callbacks._on_velocity_body
    cls._on_velocity_local = node_telemetry_callbacks._on_velocity_local
    cls._on_dvl_velocity = node_telemetry_callbacks._on_dvl_velocity
    cls._on_ground_truth_pose = node_telemetry_callbacks._on_ground_truth_pose
    cls._on_rc_out = node_telemetry_callbacks._on_rc_out
    cls._on_rc_in = node_telemetry_callbacks._on_rc_in
    cls._on_status_text = node_telemetry_callbacks._on_status_text
    cls._on_sitl_mavlink_telemetry_status = node_telemetry_callbacks._on_sitl_mavlink_telemetry_status
    cls._on_real_start_status = node_telemetry_callbacks._on_real_start_status
    cls._on_depth = node_telemetry_callbacks._on_depth
    cls._on_bar30_pressure = node_telemetry_callbacks._on_bar30_pressure
    cls._on_ping360_status = node_telemetry_callbacks._on_ping360_status
    cls._on_atm_pressure = node_telemetry_callbacks._on_atm_pressure
    cls._on_static_pressure = node_telemetry_callbacks._on_static_pressure
    cls._on_pressure_value = node_telemetry_callbacks._on_pressure_value


__all__ = [
    "bind_commanding",
    "bind_state_runtime",
    "bind_telemetry_callbacks",
    "bind_uuv_gui_node_methods",
    "bind_vehicle_info",
]
