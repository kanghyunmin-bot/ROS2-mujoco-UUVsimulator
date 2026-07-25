#!/usr/bin/env python3
"""Interactive physical Phase-homing entry point.

This is deliberately a *launch* entry point, not a shell wrapper.  It starts
the C++ ten-second frequency scanner on the already available audio stream.
The operator enters a displayed candidate number (or a frequency in Hz) in
the same terminal.  Only then is the untouched external phase estimator
started, with the selection as its immutable startup frequency, followed by
the canonical C++ RC controller.

The original hydrophone estimator has no selected-frequency subscription, so
delaying its process start is the correct contract: a selection cannot race an
already active Phase accumulator or modify the hydrophone algorithm.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Arguments owned by the non-interactive real launch.  The selected frequency
# replaces only reference_frequency_hz; every other vehicle contract and
# motion parameter is forwarded unchanged after the operator's choice.
_REAL_ARGUMENTS = (
    "dry_run",
    "use_hydrophone_estimator",
    "audio_device",
    "audio_topic",
    "audio_channels",
    "audio_sample_rate",
    "audio_sample_format",
    "audio_input_latency_s",
    "use_sim_time",
    "navigation_mode",
    "odometry_topic",
    "motion_response_enabled",
    "motion_response_velocity_topic",
    "motion_response_min_command",
    "motion_response_min_speed_mps",
    "motion_response_probe_extension_s",
    "motion_response_probe_max_extension_s",
    "motion_response_approach_grace_s",
    "motion_response_approach_hold_s",
    "motion_response_feedback_timeout_s",
    "imu_topic",
    "depth_topic",
    "state_topic",
    "delta_range_topic",
    "iq_magnitude_topic",
    "audio_quality_topic",
    "bootstrap_probe_on_audio_quality",
    "audio_quality_timeout_s",
    "direction_topic",
    "status_topic",
    "direction_output_topic",
    "rc_topic",
    "rate_hz",
    "mode",
    "auto_arm",
    "auto_mode",
    "forward_max",
    "yaw_gain",
    "yaw_command_limit",
    "tank_max_depth_m",
    "success_range_m",
    "success_hold_s",
    "arrival_radius_m",
    "arrival_hold_s",
    "first_lock_confirmation_radius_m",
    "max_runtime_s",
    "amplitude_range_constant",
    "rc_pwm_span",
    "rc_deadzone_compensation_enabled",
    "rc_xy_deadzone_pwm",
    "rc_yaw_deadzone_pwm",
    "rc_heave_deadzone_pwm",
    "probe_pwm_delta",
    "approach_pwm_delta",
    "legacy_probe_duration_scale",
    "probe_leg_s",
    "probe_neutral_s",
    "probe_settle_s",
    "probe_sample_delay_s",
    "reestimate_policy",
    "approach_min_s",
    "approach_max_s",
    "innovation_enabled",
    "innovation_window_s",
    "innovation_noise_floor_m",
    "innovation_limit",
    "innovation_hold_s",
    "innovation_min_expected_delta_m",
    "approach_duration_s",
    "initial_confirmation_probes",
)


def _gui_handoff_requested(context) -> bool:
    mode = LaunchConfiguration("gui_rc_handoff").perform(context).strip().lower()
    if mode in {"false", "0", "off", "no"}:
        return False
    if mode in {"true", "1", "on", "yes", "auto"}:
        return True
    raise RuntimeError("gui_rc_handoff must be auto, true, or false")


def _auto_select_top_requested(context) -> bool:
    value = LaunchConfiguration("auto_select_top").perform(context).strip().lower()
    if value in {"true", "1", "on", "yes"}:
        return True
    if value in {"false", "0", "off", "no"}:
        return False
    raise RuntimeError("auto_select_top must be true or false")


def _handoff_gui_rc_if_available(gate, gate_executor, context) -> None:
    """Release the optional simulator GUI publisher before the mux starts."""
    if not _gui_handoff_requested(context):
        return
    from std_srvs.srv import Trigger

    service_name = LaunchConfiguration("gui_rc_handoff_service").perform(context)
    client = gate.create_client(Trigger, service_name)
    service_ready = client.wait_for_service(timeout_sec=0.75)
    requested_mode = LaunchConfiguration("gui_rc_handoff").perform(context).strip().lower()
    if not service_ready:
        if requested_mode == "auto":
            return
        raise RuntimeError(
            f"GUI RC handoff service {service_name} is unavailable; refusing live exclusive RC launch"
        )
    future = client.call_async(Trigger.Request())
    gate_executor.spin_until_future_complete(future, timeout_sec=3.0)
    response = future.result()
    if response is None or not response.success:
        message = response.message if response is not None else "no response"
        raise RuntimeError(f"GUI RC handoff failed: {message}")
    print(f"[pinger] {response.message}")


def _restore_gui_rc_on_shutdown(context, *args, **kwargs):
    """Return simulator GUI manual control when this terminal launch exits."""
    del args, kwargs
    if not _gui_handoff_requested(context):
        return []
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger

    # This is intentionally a different service from the launch-time handoff.
    # Reusing ``.../suspend_rc_override`` here stranded the web GUI without an
    # RC publisher after a terminal-launched pinger run exited.
    service_name = LaunchConfiguration("gui_rc_restore_service").perform(context)
    gate_context = Context()
    rclpy.init(
        args=[], context=gate_context,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    gate = rclpy.create_node("pinger_frequency_launch_restore", context=gate_context)
    gate_executor = SingleThreadedExecutor(context=gate_context)
    gate_executor.add_node(gate)
    try:
        client = gate.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=0.75):
            return []
        future = client.call_async(Trigger.Request())
        gate_executor.spin_until_future_complete(future, timeout_sec=2.0)
        response = future.result()
        if response is not None:
            print(f"[pinger] GUI RC restore: {response.message}")
    finally:
        gate_executor.remove_node(gate)
        gate_executor.shutdown()
        gate.destroy_node()
        if gate_context.ok():
            rclpy.shutdown(context=gate_context)
    return []


def _start_real_homing_after_selection(context, *args, **kwargs):
    """Wait for the selector's volatile choice then include the real launch."""
    del args, kwargs
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float64, String

    selected_topic = LaunchConfiguration("selected_frequency_topic").perform(context)
    candidate_topic = LaunchConfiguration("candidate_topic").perform(context)
    manual_selection_topic = LaunchConfiguration("manual_selection_topic").perform(context)
    timeout_s = float(LaunchConfiguration("selection_timeout_s").perform(context))
    selected_frequency: list[float] = []
    candidates: list[str] = []

    gate_context = Context()
    rclpy.init(
        args=[], context=gate_context,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    gate = rclpy.create_node("pinger_frequency_launch_gate", context=gate_context)
    gate_executor = SingleThreadedExecutor(context=gate_context)
    gate_executor.add_node(gate)
    # The candidate list is transient-local: the gate can reliably receive it
    # even if terminal output is busy when the ten-second scan finishes.
    candidate_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    gate.create_subscription(String, candidate_topic, lambda message: candidates.append(message.data), candidate_qos)
    gate.create_subscription(
        Float64,
        selected_topic,
        lambda message: selected_frequency.append(float(message.data)),
        10,
    )
    selection_pub = gate.create_publisher(String, manual_selection_topic, 10)
    deadline = time.monotonic() + max(5.0, timeout_s)
    try:
        while (
            not candidates
            and time.monotonic() < deadline
            and gate_context.ok()
            and not context.is_shutdown
        ):
            gate_executor.spin_once(timeout_sec=0.20)
        if context.is_shutdown:
            return []
        if not candidates:
            raise RuntimeError(
                "No pinger frequency candidates arrived before selection_timeout_s. "
                "Verify the /audio stream and scanner band."
            )
        ranked = []
        try:
            ranked = json.loads(candidates[-1]).get("candidates", [])
            for index, candidate in enumerate(ranked, start=1):
                snr_db = float(candidate.get("snr_db", candidate["score"]))
                prominence_db = float(candidate.get("prominence_db", 0.0))
                persistence = 100.0 * float(candidate.get("persistence_ratio", 0.0))
                quality = "qualified" if bool(candidate.get("qualified", True)) else "manual-only"
                print(
                    "[pinger] "
                    f"[{index}] {float(candidate['frequency_hz']):.1f} Hz "
                    f"(hits={int(candidate['hits'])}, persistence={persistence:.0f}%, "
                    f"SNR={snr_db:.1f} dB, "
                    f"prominence={prominence_db:.1f} dB, {quality})"
                )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            print("[pinger] frequency scan complete; enter a candidate number or exact Hz.")
        auto_select_top = _auto_select_top_requested(context)
        scan_min_hz = float(LaunchConfiguration("scan_min_frequency_hz").perform(context))
        scan_max_hz = float(LaunchConfiguration("scan_max_frequency_hz").perform(context))
        if not auto_select_top:
            while True:
                choice = input("[pinger] Enter displayed candidate 1-5 or an exact frequency in Hz: ").strip()
                try:
                    numeric = float(choice)
                    valid_index = numeric.is_integer() and 1 <= int(numeric) <= len(ranked)
                    valid_frequency = scan_min_hz <= numeric <= scan_max_hz
                    if valid_index or valid_frequency:
                        break
                except ValueError:
                    pass
                print(
                    f"[pinger] Enter a displayed candidate number or "
                    f"{scan_min_hz:.0f}--{scan_max_hz:.0f} Hz."
                )
            selection_pub.publish(String(data=choice))
        else:
            print("[pinger] simulator parity run: waiting for the strongest qualified FFT candidate")
        # Candidate acquisition and operator response are separate timeout
        # phases. A slow but valid operator choice must still receive a full
        # DDS handoff window after Enter is pressed.
        selection_deadline = time.monotonic() + max(5.0, timeout_s)
        while (
            not selected_frequency
            and time.monotonic() < selection_deadline
            and gate_context.ok()
            and not context.is_shutdown
        ):
            gate_executor.spin_once(timeout_sec=0.20)
        if selected_frequency:
            _handoff_gui_rc_if_available(gate, gate_executor, context)
    finally:
        gate_executor.remove_node(gate)
        gate_executor.shutdown()
        gate.destroy_node()
        if gate_context.ok():
            rclpy.shutdown(context=gate_context)

    if context.is_shutdown:
        return []
    if not selected_frequency:
        raise RuntimeError(
            "No pinger frequency was selected before selection_timeout_s. "
            "Enter a displayed candidate number in this launch terminal."
        )
    selected_hz = selected_frequency[-1]
    scan_min_hz = float(LaunchConfiguration("scan_min_frequency_hz").perform(context))
    scan_max_hz = float(LaunchConfiguration("scan_max_frequency_hz").perform(context))
    if not scan_min_hz <= selected_hz <= scan_max_hz:
        raise RuntimeError(
            f"Selected frequency {selected_hz:.3f} Hz is outside "
            f"{scan_min_hz:.0f}--{scan_max_hz:.0f} Hz"
        )

    launch_arguments = {
        name: LaunchConfiguration(name).perform(context) for name in _REAL_ARGUMENTS
    }
    # Capture, if requested, was started before the scan.  Starting it again
    # in the included real launch would create a second physical device owner.
    launch_arguments["use_audio_capture"] = "false"
    launch_arguments["reference_frequency_hz"] = f"{selected_hz:.6f}"
    package_share = Path(get_package_share_directory("auv_pinger_homing"))
    return [
        LogInfo(msg=f"[pinger] selected {selected_hz:.3f} Hz; starting C++ Phase homing."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_share / "launch" / "pinger_homing_real.launch.py")),
            launch_arguments=launch_arguments.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("auv_pinger_homing"))
    use_audio_capture = LaunchConfiguration("use_audio_capture")

    # Keep defaults exactly aligned with pinger_homing_real.launch.py so GUI
    # fixed-frequency startup and terminal-selected startup have one vehicle
    # contract.  This launch adds only scanner/selection arguments.
    arguments = [
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("use_audio_capture", default_value="false"),
        DeclareLaunchArgument("use_hydrophone_estimator", default_value="true"),
        DeclareLaunchArgument("audio_device", default_value=""),
        DeclareLaunchArgument("audio_topic", default_value="/audio"),
        DeclareLaunchArgument("audio_channels", default_value="2"),
        DeclareLaunchArgument("audio_sample_rate", default_value="96000"),
        DeclareLaunchArgument("audio_sample_format", default_value="S32LE"),
        DeclareLaunchArgument("audio_input_latency_s", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("navigation_mode", default_value="odometry"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("motion_response_enabled", default_value="true"),
        DeclareLaunchArgument("motion_response_velocity_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("motion_response_min_command", default_value="0.05"),
        DeclareLaunchArgument("motion_response_min_speed_mps", default_value="0.03"),
        DeclareLaunchArgument("motion_response_probe_extension_s", default_value="0.30"),
        DeclareLaunchArgument("motion_response_probe_max_extension_s", default_value="1.20"),
        DeclareLaunchArgument("motion_response_approach_grace_s", default_value="0.80"),
        DeclareLaunchArgument("motion_response_approach_hold_s", default_value="0.80"),
        DeclareLaunchArgument("motion_response_feedback_timeout_s", default_value="0.75"),
        DeclareLaunchArgument("imu_topic", default_value="/mavros/imu/data"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("delta_range_topic", default_value="/audio_phase_estimator/delta_range_m"),
        DeclareLaunchArgument("iq_magnitude_topic", default_value="/audio_phase_estimator/iq_magnitude"),
        DeclareLaunchArgument("audio_quality_topic", default_value="/audio_phase_estimator/iq_snr_ratio"),
        DeclareLaunchArgument("bootstrap_probe_on_audio_quality", default_value="true"),
        DeclareLaunchArgument("audio_quality_timeout_s", default_value="3.0"),
        DeclareLaunchArgument("direction_topic", default_value="/homing/direction"),
        DeclareLaunchArgument("status_topic", default_value="/pinger_homing/status"),
        DeclareLaunchArgument("direction_output_topic", default_value="/pinger_homing/direction_body"),
        DeclareLaunchArgument("rc_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("mode", default_value="ALT_HOLD"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_mode", default_value="false"),
        DeclareLaunchArgument("forward_max", default_value="0.48"),
        DeclareLaunchArgument("yaw_gain", default_value="0.85"),
        DeclareLaunchArgument("yaw_command_limit", default_value="0.42"),
        DeclareLaunchArgument("tank_max_depth_m", default_value="11.0"),
        DeclareLaunchArgument("success_range_m", default_value="0.0"),
        DeclareLaunchArgument("success_hold_s", default_value="0.8"),
        DeclareLaunchArgument("arrival_radius_m", default_value="0.8"),
        DeclareLaunchArgument("arrival_hold_s", default_value="1.0"),
        DeclareLaunchArgument("first_lock_confirmation_radius_m", default_value="3.0"),
        DeclareLaunchArgument("max_runtime_s", default_value="180.0"),
        DeclareLaunchArgument("amplitude_range_constant", default_value="0.0"),
        DeclareLaunchArgument("rc_pwm_span", default_value="400.0"),
        DeclareLaunchArgument("rc_deadzone_compensation_enabled", default_value="true"),
        DeclareLaunchArgument("rc_xy_deadzone_pwm", default_value="30.0"),
        DeclareLaunchArgument("rc_yaw_deadzone_pwm", default_value="40.0"),
        DeclareLaunchArgument("rc_heave_deadzone_pwm", default_value="30.0"),
        DeclareLaunchArgument("probe_pwm_delta", default_value="20"),
        DeclareLaunchArgument("approach_pwm_delta", default_value="25"),
        DeclareLaunchArgument("legacy_probe_duration_scale", default_value="1.0"),
        DeclareLaunchArgument("probe_leg_s", default_value="1.5"),
        DeclareLaunchArgument("probe_neutral_s", default_value="0.50"),
        DeclareLaunchArgument("probe_settle_s", default_value="0.80"),
        DeclareLaunchArgument("probe_sample_delay_s", default_value="0.45"),
        DeclareLaunchArgument("reestimate_policy", default_value="adaptive"),
        DeclareLaunchArgument("approach_min_s", default_value="2.5"),
        DeclareLaunchArgument("approach_max_s", default_value="25.0"),
        DeclareLaunchArgument("innovation_enabled", default_value="true"),
        DeclareLaunchArgument("innovation_window_s", default_value="0.70"),
        DeclareLaunchArgument("innovation_noise_floor_m", default_value="0.0005"),
        DeclareLaunchArgument("innovation_limit", default_value="1.50"),
        DeclareLaunchArgument("innovation_hold_s", default_value="1.20"),
        DeclareLaunchArgument("innovation_min_expected_delta_m", default_value="0.0002"),
        # Compatibility-only, used only with reestimate_policy:=fixed.
        DeclareLaunchArgument("approach_duration_s", default_value="4.0"),
        DeclareLaunchArgument("initial_confirmation_probes", default_value="2"),
        DeclareLaunchArgument("selected_frequency_topic", default_value="/pinger_homing/selected_frequency_hz"),
        DeclareLaunchArgument("candidate_topic", default_value="/pinger_homing/frequency_candidates"),
        DeclareLaunchArgument("manual_selection_topic", default_value="/pinger_homing/manual_selection"),
        DeclareLaunchArgument(
            "auto_select_top",
            default_value="false",
            description=(
                "Select the strongest qualified FFT candidate without terminal input. "
                "Keep false on the physical vehicle; the simulator GUI enables it because "
                "its child launch has no interactive terminal."
            ),
        ),
        DeclareLaunchArgument("scan_monitor_s", default_value="10.0"),
        DeclareLaunchArgument("scan_min_frequency_hz", default_value="19000.0"),
        DeclareLaunchArgument("scan_max_frequency_hz", default_value="22000.0"),
        DeclareLaunchArgument("scan_combine_channels", default_value="true"),
        DeclareLaunchArgument(
            "scan_fft_size", default_value="16384",
            description=(
                "Hydrophone FFT length (96000/16384 = 5.86 Hz bins). "
                "The real profile prioritizes carrier discrimination over the "
                "simulator's faster 8192-sample scan."
            ),
        ),
        DeclareLaunchArgument(
            "scan_fft_hop_size", default_value="8192",
            description="FFT scan hop; 8192 gives 50% overlap for the 16384-sample hydrophone scan.",
        ),
        DeclareLaunchArgument(
            "scan_min_snr_db", default_value="9.0",
            description="Minimum median-noise-floor SNR for an auto-selectable candidate.",
        ),
        DeclareLaunchArgument(
            "scan_min_peak_prominence_db", default_value="4.5",
            description="Minimum local spectral-peak prominence for a qualified candidate.",
        ),
        DeclareLaunchArgument(
            "scan_minimum_candidate_hits", default_value="4",
            description="Minimum independently repeated FFT windows for a qualified candidate.",
        ),
        DeclareLaunchArgument("scan_tracking_min_snr_db", default_value="0.5"),
        DeclareLaunchArgument("scan_tracking_min_prominence_db", default_value="0.25"),
        DeclareLaunchArgument("scan_persistent_min_ratio", default_value="0.30"),
        DeclareLaunchArgument("scan_persistent_min_snr_db", default_value="1.0"),
        DeclareLaunchArgument("scan_persistent_min_prominence_db", default_value="0.5"),
        DeclareLaunchArgument(
            "scan_candidate_separation_hz", default_value="75.0",
            description="Do not list sidelobes within this distance of a stronger peak.",
        ),
        DeclareLaunchArgument(
            "scan_relative_to_top_snr_db", default_value="18.0",
            description="Hide qualified peaks this far below the strongest; set 0 to keep all.",
        ),
        DeclareLaunchArgument("selection_timeout_s", default_value="90.0"),
        DeclareLaunchArgument(
            "gui_rc_handoff",
            default_value="auto",
            description=(
                "auto calls the simulator GUI handoff service if it exists; "
                "false leaves an external physical GUI untouched."
            ),
        ),
        DeclareLaunchArgument(
            "gui_rc_handoff_service",
            default_value="/uuv_web_control_gui/suspend_rc_override",
        ),
        DeclareLaunchArgument(
            "gui_rc_restore_service",
            default_value="/uuv_web_control_gui/restore_rc_override",
            description=(
                "Optional GUI service called on launch shutdown to restore its "
                "manual RC publisher after the exclusive pinger mux exits."
            ),
        ),
    ]

    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("audio_capture")) / "launch" / "capture.launch.py")
        ),
        launch_arguments={
            "device": LaunchConfiguration("audio_device"),
            "audio_topic": LaunchConfiguration("audio_topic"),
            "channels": LaunchConfiguration("audio_channels"),
            "sample_rate": LaunchConfiguration("audio_sample_rate"),
            "sample_format": LaunchConfiguration("audio_sample_format"),
            "ns": "",
        }.items(),
        condition=IfCondition(use_audio_capture),
    )
    selector = Node(
        package="auv_pinger_homing",
        executable="pinger_frequency_selector",
        name="pinger_frequency_selector",
        output="screen",
        parameters=[{
            "audio_topic": LaunchConfiguration("audio_topic"),
            "channels": ParameterValue(LaunchConfiguration("audio_channels"), value_type=int),
            "sample_rate": ParameterValue(LaunchConfiguration("audio_sample_rate"), value_type=int),
            "monitor_s": ParameterValue(LaunchConfiguration("scan_monitor_s"), value_type=float),
            "min_frequency_hz": ParameterValue(
                LaunchConfiguration("scan_min_frequency_hz"), value_type=float
            ),
            "max_frequency_hz": ParameterValue(
                LaunchConfiguration("scan_max_frequency_hz"), value_type=float
            ),
            "combine_channels": ParameterValue(
                LaunchConfiguration("scan_combine_channels"), value_type=bool
            ),
            "fft_size": ParameterValue(LaunchConfiguration("scan_fft_size"), value_type=int),
            "fft_hop_size": ParameterValue(LaunchConfiguration("scan_fft_hop_size"), value_type=int),
            "min_snr_db": ParameterValue(LaunchConfiguration("scan_min_snr_db"), value_type=float),
            "min_peak_prominence_db": ParameterValue(
                LaunchConfiguration("scan_min_peak_prominence_db"), value_type=float
            ),
            "minimum_candidate_hits": ParameterValue(
                LaunchConfiguration("scan_minimum_candidate_hits"), value_type=int
            ),
            "tracking_min_snr_db": ParameterValue(
                LaunchConfiguration("scan_tracking_min_snr_db"), value_type=float
            ),
            "tracking_min_prominence_db": ParameterValue(
                LaunchConfiguration("scan_tracking_min_prominence_db"), value_type=float
            ),
            "persistent_min_ratio": ParameterValue(
                LaunchConfiguration("scan_persistent_min_ratio"), value_type=float
            ),
            "persistent_min_snr_db": ParameterValue(
                LaunchConfiguration("scan_persistent_min_snr_db"), value_type=float
            ),
            "persistent_min_prominence_db": ParameterValue(
                LaunchConfiguration("scan_persistent_min_prominence_db"), value_type=float
            ),
            "candidate_separation_hz": ParameterValue(
                LaunchConfiguration("scan_candidate_separation_hz"), value_type=float
            ),
            "relative_to_top_snr_db": ParameterValue(
                LaunchConfiguration("scan_relative_to_top_snr_db"), value_type=float
            ),
            "auto_select_top": ParameterValue(
                LaunchConfiguration("auto_select_top"), value_type=bool
            ),
            "selected_frequency_topic": LaunchConfiguration("selected_frequency_topic"),
            "candidate_topic": LaunchConfiguration("candidate_topic"),
            "manual_selection_topic": LaunchConfiguration("manual_selection_topic"),
            "stdin_selection_enabled": False,
        }],
    )

    return LaunchDescription(arguments + [
        capture,
        # Let a just-started capture process establish /audio before the scan.
        TimerAction(period=1.0, actions=[selector]),
        TimerAction(
            period=1.5,
            actions=[OpaqueFunction(function=_start_real_homing_after_selection)],
        ),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=_restore_gui_rc_on_shutdown)])
        ),
    ])
