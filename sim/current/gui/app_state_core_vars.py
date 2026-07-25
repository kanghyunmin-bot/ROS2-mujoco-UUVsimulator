"""Core control and vehicle-status Tk variables for the GUI."""

from __future__ import annotations

from .app_state_tk_vars import bool_vars, double_vars, string_vars


def initialize_visibility_vars(owner) -> None:
    bool_vars(
        owner,
        {
            "control_enabled": False,
            "rc_override_enabled": False,
            "control_details_visible": False,
            "vehicle_details_visible": False,
            "telemetry_visible": True,
            "control_tools_visible": True,
            "ros2_panel_visible": False,
        },
    )


def initialize_command_axis_vars(owner) -> None:
    double_vars(
        owner,
        {
            "forward_var": 0.0,
            "lateral_var": 0.0,
            "heave_var": 0.0,
            "yaw_var": 0.0,
            "rc_forward_var": 0.0,
            "rc_lateral_var": 0.0,
            "rc_heave_var": 0.0,
            "rc_yaw_var": 0.0,
        },
    )


def initialize_vehicle_status_vars(owner) -> None:
    string_vars(
        owner,
        {
            "vehicle_summary_var": "vehicle: disconnected",
            "motion_summary_var": "motion: n/a",
            "depth_target_var": "n/a",
            "depth_source_var": "depth source: unavailable",
            "mode_var": "mode: UNKNOWN",
            "status_var": "disconnected",
            "command_ready_var": "WAIT: vehicle",
            "battery_var": "battery: n/a",
            "pose_var": "pose: n/a",
            "vel_var": "velocity: n/a",
            "imu_var": "imu: n/a",
            "autopilot_var": "autopilot: n/a",
            "age_var": "state age: n/a",
            "control_summary_var": "control: idle",
            "rc_override_var": "pilot input: off",
            "control_var": "setpoint: x=0.00 y=0.00 z=0.00 yaw=0.00",
        },
    )


__all__ = [
    "initialize_command_axis_vars",
    "initialize_vehicle_status_vars",
    "initialize_visibility_vars",
]
