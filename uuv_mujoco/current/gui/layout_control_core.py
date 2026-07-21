"""Core command and stack controls for the GUI layout."""

from __future__ import annotations

from .config import GROUP_PADDING
from .layout_control_modes import build_arm_and_modes
from .layout_control_ros2 import build_ros2_buttons, build_ros2_fcu_row, build_ros2_panel
from .layout_control_stack import build_sim_stack_controls, build_telemetry_toggle
from .runtime import ttk


def build_control_panel(owner, right):
    control_box = ttk.LabelFrame(right, text="Control", padding=GROUP_PADDING)
    control_box.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
    control_box.columnconfigure(0, weight=1)

    build_telemetry_toggle(owner, control_box)
    build_sim_stack_controls(owner, control_box)
    build_ros2_panel(owner, control_box)
    build_arm_and_modes(owner, control_box)
    return control_box
