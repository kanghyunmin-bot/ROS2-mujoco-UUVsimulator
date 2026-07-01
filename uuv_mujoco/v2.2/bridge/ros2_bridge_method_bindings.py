"""Method binding table for the Ros2Bridge compatibility surface."""

from __future__ import annotations

from typing import Any

from . import (
    ros2_bridge_commands,
    ros2_bridge_init,
    ros2_bridge_public_api,
    ros2_bridge_runtime_methods,
    ros2_sitl_poll_thread,
    ros2_ping360_config,
    ros2_publish_runtime,
    ros2_sitl_sensor_feed,
    ros2_stereo_image,
    ros2_state_estimation,
)


def bind_ros2_bridge_methods(cls: type[Any]) -> None:
    """Attach split-module implementations to the historical Ros2Bridge API."""

    cls._init_ros = ros2_bridge_init.init_ros_runtime

    cls._env_to_rate_hz = ros2_bridge_runtime_methods.env_to_rate_hz
    cls._env_to_clamped_float = ros2_bridge_runtime_methods.env_to_clamped_float
    cls._is_ros_context_shutdown_error = ros2_bridge_runtime_methods.is_ros_context_shutdown_error
    cls._safe_publish = ros2_bridge_runtime_methods.safe_publish
    cls._start_ros_spin_thread = ros2_bridge_runtime_methods.start_ros_spin_thread
    cls._ros_spin_loop = ros2_bridge_runtime_methods.ros_spin_loop
    cls._start_sitl_poll_thread = ros2_sitl_poll_thread.start_sitl_poll_thread
    cls._sitl_poll_loop = ros2_sitl_poll_thread.sitl_poll_loop
    cls._stop_sitl_poll_thread = ros2_sitl_poll_thread.stop_sitl_poll_thread
    cls._sitl_poll_thread_active = ros2_sitl_poll_thread.sitl_poll_thread_active
    cls._ros_topic_due = ros2_bridge_runtime_methods.ros_topic_due
    cls._ros_imu_accel_surface = ros2_bridge_runtime_methods.ros_imu_accel_surface
    cls._sensor_slice = staticmethod(ros2_bridge_runtime_methods.sensor_slice_method)

    cls._build_mavros_state = ros2_bridge_runtime_methods.build_mavros_state
    cls._load_robot_description_text = ros2_bridge_runtime_methods.load_robot_description_text
    cls._publish_static_context = ros2_bridge_runtime_methods.publish_static_context

    cls._on_ping360_config = ros2_ping360_config.on_ping360_config

    cls._apply_cmd_deadband = ros2_bridge_commands._apply_cmd_deadband
    cls._handle_normalized_cmd = ros2_bridge_commands._handle_normalized_cmd
    cls._clear_cmd = ros2_bridge_commands._clear_cmd
    cls._on_cmd_vel_stamped = ros2_bridge_commands._on_cmd_vel_stamped
    cls._on_mavros_manual_control = ros2_bridge_commands._on_mavros_manual_control
    cls._on_mavros_rc_override = ros2_bridge_commands._on_mavros_rc_override
    cls._handle_replay_rcout_channels = ros2_bridge_commands._handle_replay_rcout_channels
    cls._on_replay_rcout_override = ros2_bridge_commands._on_replay_rcout_override
    cls._on_sitl_servo_output_for_ros = ros2_bridge_commands._on_sitl_servo_output_for_ros
    cls._on_mavros_setpoint = ros2_bridge_commands._on_mavros_setpoint
    cls._parse_command_bool = staticmethod(ros2_bridge_commands._parse_command_bool)
    cls._parse_command_override_payload = staticmethod(ros2_bridge_commands._parse_command_override_payload)
    cls._forward_arm_request = ros2_bridge_commands._forward_arm_request
    cls._forward_mode_request = ros2_bridge_commands._forward_mode_request
    cls._on_sitl_command_override = ros2_bridge_commands._on_sitl_command_override
    cls._on_mavros_cmd_arming = ros2_bridge_commands._on_mavros_cmd_arming
    cls._on_mavros_set_mode = ros2_bridge_commands._on_mavros_set_mode
    cls._on_mavros_command_long = ros2_bridge_commands._on_mavros_command_long

    cls.set_sitl_initial_depth_hold_active = (
        ros2_state_estimation.set_sitl_initial_depth_hold_active
    )
    cls._sitl_vertical_feedback_zero_reason = (
        ros2_state_estimation._sitl_vertical_feedback_zero_reason
    )
    cls._log_sitl_vertical_feedback_zero = ros2_state_estimation._log_sitl_vertical_feedback_zero
    cls._estimate_base_accel_enu = ros2_state_estimation._estimate_base_accel_enu
    cls._estimate_vertical_truth = ros2_state_estimation._estimate_vertical_truth
    cls._estimate_bar30_pressure_pa = ros2_state_estimation._estimate_bar30_pressure_pa
    cls._site_world_pos_enu = ros2_state_estimation._site_world_pos_enu
    cls._body_cvel_world_linear_velocity_enu = (
        ros2_state_estimation._body_cvel_world_linear_velocity_enu
    )
    cls._object_world_linear_velocity_enu = (
        ros2_state_estimation._object_world_linear_velocity_enu
    )
    cls._estimate_sitl_vertical = ros2_state_estimation._estimate_sitl_vertical
    cls._imu_vectors_in_body = ros2_state_estimation._imu_vectors_in_body
    cls._specific_force_body = ros2_state_estimation._specific_force_body
    cls._dvl_velocity_body = ros2_state_estimation._dvl_velocity_body
    cls._apply_mavros_setpoint = ros2_state_estimation._apply_mavros_setpoint
    cls._build_and_send_sitl_sensor_snapshot = (
        ros2_sitl_sensor_feed.build_and_send_sitl_sensor_snapshot
    )
    cls._flush_ros_publish_jobs = ros2_publish_runtime.flush_ros_publish_jobs
    cls.render_camera_rgb = ros2_stereo_image.render_camera_rgb
    cls.can_share_camera_renderer = ros2_stereo_image.can_share_camera_renderer
    cls._close_stereo_image_renderers = ros2_stereo_image.close_stereo_image_renderers

    cls.set_sitl_servo_handler = ros2_bridge_public_api.set_sitl_servo_handler
    cls.set_replay_rcout_handler = ros2_bridge_public_api.set_replay_rcout_handler
    cls.sitl_vehicle_armed = ros2_bridge_public_api.sitl_vehicle_armed
    cls.sitl_vehicle_mode = ros2_bridge_public_api.sitl_vehicle_mode
    cls.spin_once = ros2_bridge_public_api.spin_once
    cls.publish = ros2_bridge_public_api.publish
    cls.force_next_publish = ros2_bridge_public_api.force_next_publish
    cls.reset_odometry = ros2_bridge_public_api.reset_odometry
    cls.shutdown = ros2_bridge_public_api.shutdown


__all__ = ["bind_ros2_bridge_methods"]
