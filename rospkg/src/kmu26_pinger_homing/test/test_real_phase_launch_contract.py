#!/usr/bin/env python3
"""Keep the physical launch on the canonical C++ Phase path.

The simulator success path and the physical launch must not silently diverge
back to the archived Python controller.  This is a source-level contract test:
hardware is intentionally not touched.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_real_launch_uses_canonical_cpp_odometry_controller() -> None:
    launch = (ROOT / "launch" / "pinger_homing_real.launch.py").read_text()
    assert 'executable="pinger_homing_controller"' in launch
    assert 'executable="single_hydrophone_homing_controller.py"' not in launch
    assert 'default_value="odometry"' in launch
    assert '"navigation_mode": LaunchConfiguration("navigation_mode")' in launch
    assert 'DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered")' in launch
    assert '"acoustic_estimator_mode": "phase"' in launch
    assert '"no_odom_horizontal_only": True' in launch
    assert '"no_odom_vertical_control_enabled": False' in launch
    assert '"motion_response_enabled"' in launch
    assert '"motion_response_velocity_topic"' in launch
    assert '"motion_response_min_speed_mps"' in launch
    assert '"probe_pwm_delta": ParameterValue(' in launch
    assert '"approach_pwm_delta": ParameterValue(' in launch
    assert '"rc_deadzone_compensation_enabled": ParameterValue(' in launch
    assert '"rc_xy_deadzone_pwm": ParameterValue(' in launch
    assert '"rc_yaw_deadzone_pwm": ParameterValue(' in launch
    assert '"rc_heave_deadzone_pwm": ParameterValue(' in launch
    assert '"probe_duration_scale": ParameterValue(' in launch
    assert '"rc_output_topic": rc_topic' in launch
    assert 'executable="rc_override_mux"' not in launch


def test_real_launch_preserves_external_hydrophone_estimator_boundary() -> None:
    launch = (ROOT / "launch" / "pinger_homing_real.launch.py").read_text()
    assert 'package="audio_capture"' in launch
    assert 'executable="audio_phase_estimator"' in launch
    assert '"enable_frequency_acquisition": False' in launch
    assert '"use_sim_time"' in launch
    assert '"audio_input_latency_s"' in launch
    assert '"audio_quality_topic"' in launch
    assert '"bootstrap_probe_on_audio_quality"' in launch


def test_xy_alt_hold_does_not_require_depth_for_a_heave_free_probe() -> None:
    controller = (ROOT / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    assert "passive_vertical_alt_hold" in controller
    assert "no_odom_horizontal_only_ && !no_odom_vertical_control_enabled_" in controller


def test_interactive_launch_scans_then_injects_selected_startup_frequency() -> None:
    launch = (ROOT / "launch" / "pinger_homing_real_interactive.launch.py").read_text()
    assert 'executable="pinger_frequency_selector"' in launch
    assert "OpaqueFunction(function=_start_real_homing_after_selection)" in launch
    assert '"selected_frequency_topic"' in launch
    assert 'launch_arguments["reference_frequency_hz"]' in launch
    assert 'launch_arguments["use_audio_capture"] = "false"' in launch
    assert 'gui_rc_handoff' in launch
    assert '"scan_fft_size", default_value="16384"' in launch
    assert '"scan_fft_hop_size", default_value="8192"' in launch
    assert '"scan_monitor_s", default_value="10.0"' in launch
    assert '"scan_min_frequency_hz", default_value="19000.0"' in launch
    assert '"scan_max_frequency_hz", default_value="22000.0"' in launch
    assert '"scan_combine_channels", default_value="true"' in launch
    assert '"scan_persistent_min_ratio", default_value="0.30"' in launch
    assert '"auto_select_top",' in launch
    assert 'default_value="false"' in launch
    assert 'DeclareLaunchArgument("rc_deadzone_compensation_enabled", default_value="true")' in launch
    assert 'DeclareLaunchArgument("rc_xy_deadzone_pwm", default_value="30.0")' in launch
    assert 'DeclareLaunchArgument("rc_yaw_deadzone_pwm", default_value="40.0")' in launch
    assert "_auto_select_top_requested(context)" in launch
    assert 'LaunchConfiguration("auto_select_top"), value_type=bool' in launch
    assert "if not auto_select_top:" in launch
    assert "96000/16384 = 5.86 Hz bins" in launch
    assert "args=[], context=gate_context" in launch
    assert "SignalHandlerOptions.NO" in launch
    assert "not context.is_shutdown" in launch
    assert "SingleThreadedExecutor(context=gate_context)" in launch
    assert "gate_executor.spin_once" in launch
    assert "selection_deadline = time.monotonic()" in launch
    assert '"use_rc_mux"' not in launch
    assert 'gui_rc_handoff_service' in launch
    assert 'gui_rc_restore_service' in launch
    assert 'default_value="/uuv_web_control_gui/suspend_rc_override"' in launch
    assert 'default_value="/uuv_web_control_gui/restore_rc_override"' in launch
    restore_body = launch.split("def _restore_gui_rc_on_shutdown", 1)[1].split(
        "def _start_real_homing_after_selection", 1
    )[0]
    assert 'LaunchConfiguration("gui_rc_restore_service")' in restore_body
    assert 'LaunchConfiguration("gui_rc_handoff_service")' not in restore_body
    assert 'OnShutdown' in launch
    assert '"motion_response_enabled"' in launch
    assert not (ROOT / "scripts" / "start_pinger_homing_real.sh").exists()


def test_motion_response_adapts_timing_without_becoming_phase_bearing_input() -> None:
    controller = (ROOT / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    assert "handle_no_odom_motion_response" in controller
    assert "motion_response_velocity_topic" in controller


def test_rc_pwm_contract_compensates_physical_pixhawk_deadzone() -> None:
    controller = (ROOT / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    assert "effective_delta + std::max(0.0, deadzone_pwm)" in controller
    assert "if (std::abs(command) <= 1.0e-9) return kRcNeutral" in controller
    assert '"rc_deadzone_compensation_enabled", true' in controller
    assert '"rc_xy_deadzone_pwm", 30.0' in controller
    assert '"rc_yaw_deadzone_pwm", 40.0' in controller


def test_uncalibrated_near_first_fit_requires_mirrored_confirmation() -> None:
    controller = (ROOT / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    real_launch = (ROOT / "launch" / "pinger_homing_real.launch.py").read_text()
    interactive = (ROOT / "launch" / "pinger_homing_real_interactive.launch.py").read_text()
    assert '"first_lock_confirmation_radius_m", 3.0' in controller
    assert "near_first_fit_needs_confirmation" in controller
    assert "requiring mirrored confirmation before lock" in controller
    assert 'DeclareLaunchArgument("first_lock_confirmation_radius_m", default_value="3.0")' in real_launch
    assert '"first_lock_confirmation_radius_m"' in interactive


def test_adaptive_phase_reestimate_uses_innovation_not_fixed_cadence() -> None:
    root = Path(__file__).resolve().parents[1]
    controller = (root / "src" / "pinger_homing" / "pinger_homing_controller.cpp").read_text()
    real_launch = (root / "launch" / "pinger_homing_real.launch.py").read_text()
    tank_launch = (root / "launch" / "pinger_homing_test_tank.launch.py").read_text()
    assert '"no_odom_reestimate_policy", "adaptive"' in controller
    assert "handle_no_odom_phase_innovation" in controller
    assert "handle_no_odom_approach_watchdog" in controller
    assert "no_odom_innovation_limit" in controller
    assert 'DeclareLaunchArgument("reestimate_policy", default_value="adaptive")' in real_launch
    assert 'DeclareLaunchArgument("approach_max_s", default_value="25.0")' in real_launch
    assert 'DeclareLaunchArgument("reestimate_policy", default_value="adaptive")' in tank_launch
    assert "no_odom_probe_leg_extensions_s_" in controller
    assert "uses_for_bearing\\\":false" in controller
    assert "never enter the Phase ABBA regression" in controller


if __name__ == "__main__":
    test_real_launch_uses_canonical_cpp_odometry_controller()
    test_real_launch_preserves_external_hydrophone_estimator_boundary()
    test_xy_alt_hold_does_not_require_depth_for_a_heave_free_probe()
    test_interactive_launch_scans_then_injects_selected_startup_frequency()
    test_motion_response_adapts_timing_without_becoming_phase_bearing_input()
    test_rc_pwm_contract_compensates_physical_pixhawk_deadzone()
    test_uncalibrated_near_first_fit_requires_mirrored_confirmation()
    test_adaptive_phase_reestimate_uses_innovation_not_fixed_cadence()
