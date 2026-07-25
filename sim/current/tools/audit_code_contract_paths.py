"""Source path registry for source-level contract audits."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import ACTIVE_RUNTIME_ROOT, ARDUPILOT_ROOT


def runtime_path(relative_path: str) -> Path:
    return ACTIVE_RUNTIME_ROOT / relative_path


def ardupilot_source_paths() -> dict[str, Path]:
    return {
        "sim_json_h": ARDUPILOT_ROOT / "libraries/SITL/SIM_JSON.h",
        "sim_json_cpp": ARDUPILOT_ROOT / "libraries/SITL/SIM_JSON.cpp",
        "baro_sitl_cpp": ARDUPILOT_ROOT / "libraries/AP_Baro/AP_Baro_SITL.cpp",
        "gcs_common_cpp": ARDUPILOT_ROOT / "libraries/GCS_MAVLink/GCS_Common.cpp",
        "rc_channel_cpp": ARDUPILOT_ROOT / "libraries/RC_Channel/RC_Channel.cpp",
        "rc_varinfo_h": ARDUPILOT_ROOT / "libraries/RC_Channel/RC_Channels_VarInfo.h",
        "joystick_cpp": ARDUPILOT_ROOT / "ArduSub/joystick.cpp",
        "aircraft_cpp": ARDUPILOT_ROOT / "libraries/SITL/SIM_Aircraft.cpp",
        "motors6dof_cpp": ARDUPILOT_ROOT / "libraries/AP_Motors/AP_Motors6DOF.cpp",
    }


def active_runtime_paths() -> dict[str, Path]:
    return {
        "sitl_contract_py": runtime_path("bridge/sitl_contract.py"),
        "baro_contract_py": runtime_path("sim/contracts/baro.py"),
        "sitl_json_sensor_runtime_py": runtime_path("bridge/sitl_json_payload.py"),
        "ros2_publish_runtime_py": runtime_path("bridge/ros2_publish_runtime.py"),
        "ros2_publish_state_py": runtime_path("bridge/ros2_publish_state.py"),
        "ros2_publish_core_factories_py": runtime_path("bridge/ros2_publish_core_factories.py"),
        "ros2_publish_dvl_factories_py": runtime_path("bridge/ros2_publish_dvl_factories.py"),
        "ros2_publish_mavros_cache_factories_py": runtime_path(
            "bridge/ros2_publish_mavros_cache_factories.py"
        ),
        "ros2_publish_mavros_cache_imu_py": runtime_path("bridge/ros2_publish_mavros_cache_imu.py"),
        "ros2_publish_mavros_cache_status_py": runtime_path("bridge/ros2_publish_mavros_cache_status.py"),
        "ros2_sitl_sensor_feed_py": runtime_path("bridge/ros2_sitl_sensor_feed.py"),
        "ros2_sitl_sensor_transport_py": runtime_path("bridge/ros2_sitl_sensor_transport.py"),
        "ros2_state_estimation_py": runtime_path("bridge/ros2_state_estimation.py"),
        "ros2_state_sitl_vertical_py": runtime_path("bridge/ros2_state_sitl_vertical.py"),
        "ros2_state_sitl_baro_py": runtime_path("bridge/ros2_state_sitl_baro.py"),
        "ros2_state_vertical_py": runtime_path("bridge/ros2_state_vertical.py"),
        "ros2_bridge_config_py": runtime_path("bridge/ros2_bridge_config.py"),
        "ros2_bridge_config_pressure_py": runtime_path("bridge/ros2_bridge_config_pressure.py"),
        "ros2_bridge_config_baro_py": runtime_path("bridge/ros2_bridge_config_baro.py"),
        "ros2_bridge_config_imu_py": runtime_path("bridge/ros2_bridge_config_imu.py"),
        "ros2_bridge_config_static_pressure_py": runtime_path("bridge/ros2_bridge_config_static_pressure.py"),
        "ros2_bridge_config_vertical_py": runtime_path("bridge/ros2_bridge_config_vertical.py"),
        "ros2_bridge_publish_timing_py": runtime_path("bridge/ros2_bridge_publish_timing.py"),
        "sitl_sensor_replay_clock_py": runtime_path("bridge/sitl_sensor_replay_clock.py"),
        "sitl_sensor_replay_servo_clock_py": runtime_path("bridge/sitl_sensor_replay_servo_clock.py"),
        "sitl_sensor_replay_payload_time_py": runtime_path("bridge/sitl_sensor_replay_payload_time.py"),
        "ros2_rc_override_callback_py": runtime_path("bridge/ros2_rc_override_callback.py"),
        "ros2_rc_override_forwarding_py": runtime_path("bridge/ros2_rc_override_forwarding.py"),
        "ros2_rc_override_frame_py": runtime_path("bridge/ros2_rc_override_frame.py"),
        "ros2_rc_override_mirror_py": runtime_path("bridge/ros2_rc_override_mirror.py"),
        "sitl_json_servo_poll_loop_py": runtime_path("bridge/sitl_json_servo_poll_loop.py"),
        "sitl_mavlink_request_servo_py": runtime_path("bridge/sitl_mavlink_request_servo.py"),
        "sitl_mavlink_request_servo_link_py": runtime_path("bridge/sitl_mavlink_request_servo_link.py"),
        "sitl_mavlink_request_command_link_py": runtime_path("bridge/sitl_mavlink_request_command_link.py"),
        "sitl_transport_extnav_runtime_py": runtime_path("bridge/sitl_transport_extnav_runtime.py"),
        "sitl_transport_extnav_scheduler_py": runtime_path("bridge/sitl_transport_extnav_scheduler.py"),
        "sitl_pwm_frame_handler_py": runtime_path("bridge/sitl_pwm_frame_handler.py"),
        "sitl_pwm_source_policy_py": runtime_path("bridge/sitl_pwm_source_policy.py"),
        "dynamic_fluidcoef_loads_py": runtime_path("sim/physics/dynamic_fluidcoef_loads.py"),
        "dynamic_fluidcoef_pattern_prepare_py": runtime_path("sim/physics/dynamic_fluidcoef_pattern_prepare.py"),
        "dynamic_fluidcoef_runtime_py": runtime_path("sim/physics/dynamic_fluidcoef_runtime.py"),
        "dynamic_fluidcoef_runtime_config_py": runtime_path("sim/physics/dynamic_fluidcoef_runtime_config.py"),
        "dynamic_fluidcoef_runtime_update_py": runtime_path("sim/physics/dynamic_fluidcoef_runtime_update.py"),
        "dynamic_fluidcoef_setup_config_py": runtime_path("sim/physics/dynamic_fluidcoef_setup_config.py"),
        "dynamic_fluidcoef_setup_enable_py": runtime_path("sim/physics/dynamic_fluidcoef_setup_enable.py"),
        "dynamic_fluidcoef_setup_rows_py": runtime_path("sim/physics/dynamic_fluidcoef_setup_rows.py"),
        "hydrodynamics_runtime_current_py": runtime_path("sim/runtime/hydrodynamics_runtime_current.py"),
        "model_runtime_setup_py": runtime_path("sim/runtime/model_runtime_setup.py"),
        "underwater_wrench_runtime_py": runtime_path("sim/runtime/underwater_wrench_runtime.py"),
        "sim_profiles_json": runtime_path("config/sim_profiles.json"),
        "simulation_loop_clocks_py": runtime_path("sim/runtime/simulation_loop_clocks.py"),
        "simulation_loop_catchup_py": runtime_path("sim/runtime/simulation_loop_catchup.py"),
        "simulation_step_catchup_py": runtime_path("sim/runtime/simulation_step_catchup.py"),
        "simulation_sensor_catchup_py": runtime_path("sim/runtime/simulation_sensor_catchup.py"),
        "simulation_step_runtime_py": runtime_path("sim/runtime/simulation_step_runtime.py"),
        "simulation_step_raw_pwm_py": runtime_path("sim/runtime/simulation_step_raw_pwm.py"),
        "simulation_step_physics_py": runtime_path("sim/runtime/simulation_step_physics.py"),
        "physics_step_thruster_callbacks_py": runtime_path("sim/runtime/physics_step_thruster_callbacks.py"),
        "hydrodynamics_runtime_setup_py": runtime_path("sim/runtime/hydrodynamics_runtime_setup.py"),
        "thruster_actuator_runtime_py": runtime_path("sim/runtime/thruster_actuator_runtime.py"),
        "thruster_actuator_command_py": runtime_path("sim/runtime/thruster_actuator_command.py"),
        "thruster_actuator_forces_py": runtime_path("sim/runtime/thruster_actuator_forces.py"),
        "thruster_actuator_immersion_py": runtime_path("sim/runtime/thruster_actuator_immersion.py"),
        "thruster_actuator_wrench_py": runtime_path("sim/runtime/thruster_actuator_wrench.py"),
        "thruster_force_model_py": runtime_path("sim/physics/thruster_force_model.py"),
        "thruster_force_performance_py": runtime_path("sim/physics/thruster_force_performance.py"),
        "thruster_force_polynomial_py": runtime_path("sim/physics/thruster_force_polynomial.py"),
        "thruster_mapping_py": runtime_path("physics/thruster_mapping.py"),
    }
