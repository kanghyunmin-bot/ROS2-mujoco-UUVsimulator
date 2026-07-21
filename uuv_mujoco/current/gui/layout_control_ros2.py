"""ROS2/MAVROS utility controls for the GUI control panel."""

from __future__ import annotations

from .config import INNER_PADDING, ROS_PACKAGE_DIR
from .runtime import tk, ttk


def build_ros2_panel(owner, control_box) -> None:
    ros2_toggle_row = ttk.Frame(control_box)
    ros2_toggle_row.grid(row=2, column=0, sticky="ew", pady=(0, 4))
    ros2_toggle_row.columnconfigure(0, weight=1)
    ttk.Label(ros2_toggle_row, text="ROS2 sim MAVROS / read-only rospkg").grid(row=0, column=0, sticky="w")
    owner.ros2_panel_button = ttk.Button(
        ros2_toggle_row,
        text="ROS2 Panel",
        style="Danger.TButton",
        command=owner._toggle_ros2_panel,
    )
    owner.ros2_panel_button.grid(row=0, column=1, sticky="e")

    owner.ros2_panel_frame = ttk.LabelFrame(
        control_box,
        text="ROS2 MAVROS / RViz",
        padding=INNER_PADDING,
        style="Danger.TLabelframe",
    )
    owner.ros2_panel_frame.grid(row=3, column=0, sticky="ew", pady=(0, 4))
    owner.ros2_panel_frame.columnconfigure(0, weight=1)
    ttk.Label(owner.ros2_panel_frame, text=f"rospkg read-only source: {ROS_PACKAGE_DIR}", anchor="w").grid(
        row=0, column=0, columnspan=3, sticky="ew"
    )
    build_ros2_fcu_row(owner)
    build_ros2_buttons(owner)
    owner.ros2_panel_frame.grid_remove()


def build_ros2_fcu_row(owner) -> None:
    fcu_row = ttk.Frame(owner.ros2_panel_frame)
    fcu_row.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(3, 0))
    fcu_row.columnconfigure(1, weight=1)
    ttk.Label(fcu_row, text="fcu_url").grid(row=0, column=0, sticky="w", padx=(0, 4))
    ttk.Entry(fcu_row, textvariable=owner.ros_pkg_fcu_url_var).grid(row=0, column=1, sticky="ew")


def build_ros2_buttons(owner) -> None:
    ros2_buttons = ttk.Frame(owner.ros2_panel_frame)
    ros2_buttons.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(4, 0))
    ttk.Button(ros2_buttons, text="Build pkg (Ubuntu)", command=owner._build_ros_pkg).pack(side=tk.LEFT)
    owner.mavros_toggle_button = ttk.Button(
        ros2_buttons,
        text="MAVROS ON",
        style="Danger.TButton",
        command=owner._toggle_ros_pkg_stack,
    )
    owner.mavros_toggle_button.pack(side=tk.LEFT, padx=(4, 0))
    owner.rviz_toggle_button = ttk.Button(
        ros2_buttons,
        text="RViz ON",
        style="Danger.TButton",
        command=owner._toggle_rviz,
    )
    owner.rviz_toggle_button.pack(side=tk.LEFT, padx=(4, 0))
    ttk.Label(owner.ros2_panel_frame, textvariable=owner.ros_pkg_status_var, anchor="w", style="Status.TLabel").grid(
        row=3, column=0, columnspan=3, sticky="ew", pady=(3, 0)
    )
    ttk.Label(owner.ros2_panel_frame, textvariable=owner.rviz_status_var, anchor="w", style="Status.TLabel").grid(
        row=4, column=0, columnspan=3, sticky="ew"
    )
