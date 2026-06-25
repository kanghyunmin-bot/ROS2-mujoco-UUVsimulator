"""Feature-specific Tk variables for the UUV control GUI."""

from __future__ import annotations

from .app_state_tk_vars import bool_vars, string_vars, tk
from .config import (
    DEFAULT_RC_REPLAY_BAG,
    PHYSICS_PARAM_SPECS,
    ROS_PACKAGE_DEFAULT_FCU_URL,
)


def initialize_replay_vars(owner) -> None:
    string_vars(
        owner,
        {
            "rc_replay_path_var": str(DEFAULT_RC_REPLAY_BAG),
            "rc_replay_rate_var": "1.0",
            "rc_replay_status_var": "replay: unloaded",
            "rc_replay_time_var": "00:00.0 / 00:00.0",
        },
    )
    owner.rc_replay_position_var = tk.DoubleVar(value=0.0)


def initialize_physics_vars(owner) -> None:
    owner.physics_status_var = tk.StringVar(value="physics params: idle")
    owner.buoy_layout_status_var = tk.StringVar(value="course layout: idle")
    owner.physics_param_vars = {
        str(spec["key"]): tk.StringVar(value="")
        for spec in PHYSICS_PARAM_SPECS
    }


def initialize_runtime_panel_vars(owner) -> None:
    string_vars(
        owner,
        {
            "sim_stack_status_var": "sim: stopped",
            "ros_pkg_status_var": "mavros: stopped",
            "ros_pkg_fcu_url_var": ROS_PACKAGE_DEFAULT_FCU_URL,
            "rviz_status_var": "rviz: stopped",
        },
    )


def initialize_ping360_vars(owner) -> None:
    string_vars(
        owner,
        {
            "ping360_view_status_var": "ping360 view: closed",
            "ping360_range_var": "2.0",
            "ping360_num_steps_var": "1",
            "ping360_gain_var": "0",
            "ping360_interface_var": "ethernet",
            "ping360_frequency_var": "750",
            "ping360_start_angle_var": "0",
            "ping360_stop_angle_var": "399",
            "ping360_summary_var": "ping360: no status",
        },
    )
    owner.ping360_enabled_var = tk.BooleanVar(value=False)


__all__ = [
    "initialize_physics_vars",
    "initialize_ping360_vars",
    "initialize_replay_vars",
    "initialize_runtime_panel_vars",
]
