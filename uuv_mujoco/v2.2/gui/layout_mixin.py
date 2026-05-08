"""Tk layout construction for the MuJoCo UUV GUI."""

from __future__ import annotations

import datetime as _dt

from .config import *
from .helpers import *
from .models import ControlCommands, RcReplaySample
from .node import UuvGuiNode
from .ros_tools import *
from .runtime import *
from .theme import apply_theme
from .widgets import VirtualJoystick

class LayoutMixin:
    def _build_layout(self) -> None:
        style = ttk.Style(self.root)
        apply_theme(self.root, style)

        container = ttk.Frame(self.root, padding=OUTER_PADDING, style="App.TFrame")
        self.main_container = container
        container.pack(fill=tk.BOTH, expand=True)
        container.columnconfigure(0, weight=3)
        container.columnconfigure(1, weight=2)
        container.rowconfigure(1, weight=1)

        header = ttk.Frame(container, style="Header.TFrame", padding=(12, 9))
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        header.columnconfigure(0, weight=1)
        title_stack = ttk.Frame(header, style="Header.TFrame")
        title_stack.grid(row=0, column=0, sticky="w")
        ttk.Label(title_stack, text="UUV Control GUI", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(title_stack, text="MuJoCo, ROS2, MAVROS, RViz, Ping360", style="Subtitle.TLabel").grid(
            row=1, column=0, sticky="w", pady=(1, 0)
        )
        status_strip = ttk.Frame(header, style="Header.TFrame")
        status_strip.grid(row=0, column=1, sticky="e")
        for idx, var in enumerate((self.sim_stack_status_var, self.ros_pkg_status_var, self.ping360_summary_var)):
            ttk.Label(status_strip, textvariable=var, style="StatusPill.TLabel").grid(
                row=0, column=idx, sticky="e", padx=(6 if idx else 0, 0)
            )

        left = ttk.Frame(container)
        self.telemetry_panel = left
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        left.rowconfigure(2, weight=1)

        right = ttk.Frame(container)
        self.control_panel = right
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)

        summary = ttk.LabelFrame(left, text="Vehicle Summary", padding=GROUP_PADDING)
        summary.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        summary.columnconfigure(0, weight=1)

        summary_header = ttk.Frame(summary)
        summary_header.grid(row=0, column=0, sticky="ew")
        summary_header.columnconfigure(0, weight=1)
        ttk.Label(
            summary_header,
            textvariable=self.vehicle_summary_var,
            anchor="w",
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=0, column=0, sticky="ew")
        self.vehicle_details_button = ttk.Button(
            summary_header,
            text="Details >",
            width=10,
            command=self._toggle_vehicle_details,
        )
        self.vehicle_details_button.grid(row=0, column=1, sticky="e", padx=(6, 0))

        ttk.Label(summary, textvariable=self.motion_summary_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=(2, 0)
        )
        ttk.Label(summary, textvariable=self.control_summary_var, anchor="w").grid(
            row=2, column=0, sticky="ew", pady=(1, 0)
        )

        self.vehicle_details_frame = ttk.Frame(summary)
        self.vehicle_details_frame.grid(row=3, column=0, sticky="ew", pady=(4, 0))
        self.vehicle_details_frame.columnconfigure(0, weight=1)
        for row, var in enumerate(
            (
                self.status_var,
                self.mode_var,
                self.battery_var,
                self.pose_var,
                self.vel_var,
                self.imu_var,
                self.autopilot_var,
                self.depth_target_var,
                self.depth_source_var,
                self.age_var,
            )
        ):
            ttk.Label(self.vehicle_details_frame, textvariable=var, anchor="w").grid(
                row=row, column=0, sticky="ew", pady=1
            )
        self.vehicle_details_frame.grid_remove()

        visuals = ttk.Frame(left)
        visuals.grid(row=1, column=0, sticky="nsew", pady=(0, 6))
        visuals.columnconfigure(0, weight=3)
        visuals.columnconfigure(1, weight=2)
        visuals.rowconfigure(0, weight=1)

        attitude_box = ttk.LabelFrame(visuals, text="Attitude", padding=GROUP_PADDING)
        attitude_box.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.attitude_canvas = tk.Canvas(
            attitude_box,
            width=ATTITUDE_CANVAS_WIDTH,
            height=ATTITUDE_CANVAS_HEIGHT,
            bg="#0f172a",
            highlightthickness=0,
        )
        self.attitude_canvas.pack(fill=tk.BOTH, expand=True)

        side_box = ttk.Frame(visuals)
        side_box.grid(row=0, column=1, sticky="nsew")
        side_box.rowconfigure(0, weight=1)
        side_box.rowconfigure(1, weight=1)
        side_box.columnconfigure(0, weight=1)

        depth_box = ttk.LabelFrame(side_box, text="Depth", padding=GROUP_PADDING)
        depth_box.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        self.depth_canvas = tk.Canvas(
            depth_box,
            width=DEPTH_CANVAS_WIDTH,
            height=DEPTH_CANVAS_HEIGHT,
            bg="#081018",
            highlightthickness=0,
        )
        self.depth_canvas.pack(fill=tk.BOTH, expand=True)

        rc_box = ttk.LabelFrame(side_box, text="RC Feedback", padding=GROUP_PADDING)
        rc_box.grid(row=1, column=0, sticky="nsew")
        self._rc_bars = []
        self._rc_labels = []
        for idx in range(RC_VISIBLE_CHANNEL_COUNT):
            ttk.Label(rc_box, text=f"Ch {idx + 1:02d}").grid(row=idx, column=0, sticky="w")
            bar = ttk.Progressbar(
                rc_box,
                orient=tk.HORIZONTAL,
                maximum=800,
                mode="determinate",
                style="Telemetry.Horizontal.TProgressbar",
            )
            bar.grid(row=idx, column=1, sticky="ew", padx=6)
            value_label = ttk.Label(rc_box, text="0")
            value_label.grid(row=idx, column=2, sticky="e")
            self._rc_bars.append(bar)
            self._rc_labels.append(value_label)
        rc_box.columnconfigure(1, weight=1)

        log_box = ttk.LabelFrame(left, text="Events", padding=GROUP_PADDING)
        log_box.grid(row=2, column=0, sticky="nsew")
        self.event_list = tk.Listbox(
            log_box,
            activestyle="none",
            bg="#ffffff",
            fg="#0f172a",
            selectbackground="#dbeafe",
            selectforeground="#0f172a",
            highlightthickness=1,
            highlightbackground="#cbd5e1",
            borderwidth=0,
            font=("TkDefaultFont", 10),
        )
        self.event_list.pack(fill=tk.BOTH, expand=True)

        control_box = ttk.LabelFrame(right, text="Control", padding=GROUP_PADDING)
        control_box.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        control_box.columnconfigure(0, weight=1)

        telemetry_row = ttk.Frame(control_box)
        telemetry_row.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        telemetry_row.columnconfigure(0, weight=1)
        ttk.Label(telemetry_row, text="Telemetry panel").grid(row=0, column=0, sticky="w")
        self.telemetry_toggle_button = ttk.Button(
            telemetry_row,
            text="Hide telemetry",
            command=self._toggle_telemetry_panel,
        )
        self.telemetry_toggle_button.grid(row=0, column=1, sticky="e")

        stack_row = ttk.LabelFrame(control_box, text="Simulation Stack", padding=INNER_PADDING)
        stack_row.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        stack_row.columnconfigure(0, weight=1)
        stack_buttons = ttk.Frame(stack_row)
        stack_buttons.grid(row=0, column=0, sticky="ew")
        ttk.Button(
            stack_buttons,
            text="Start SITL/MuJoCo",
            style="Success.TButton",
            command=self._start_sim_stack,
        ).pack(side=tk.LEFT)
        ttk.Button(
            stack_buttons,
            text="Stop/Reset",
            style="Danger.TButton",
            command=self._stop_sim_stack,
        ).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(
            stack_buttons,
            text="Ping360 panel",
            style="Info.TButton",
            command=self._toggle_ping360_window,
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Label(stack_row, textvariable=self.sim_stack_status_var, anchor="w", style="Status.TLabel").grid(
            row=1, column=0, sticky="ew", pady=(2, 0)
        )

        ros2_toggle_row = ttk.Frame(control_box)
        ros2_toggle_row.grid(row=2, column=0, sticky="ew", pady=(0, 4))
        ros2_toggle_row.columnconfigure(0, weight=1)
        ttk.Label(ros2_toggle_row, text="ROS2 sim MAVROS / read-only rospkg").grid(row=0, column=0, sticky="w")
        self.ros2_panel_button = ttk.Button(
            ros2_toggle_row,
            text="ROS2 Panel",
            style="Danger.TButton",
            command=self._toggle_ros2_panel,
        )
        self.ros2_panel_button.grid(row=0, column=1, sticky="e")

        self.ros2_panel_frame = ttk.LabelFrame(
            control_box,
            text="ROS2 MAVROS / RViz",
            padding=INNER_PADDING,
            style="Danger.TLabelframe",
        )
        self.ros2_panel_frame.grid(row=3, column=0, sticky="ew", pady=(0, 4))
        self.ros2_panel_frame.columnconfigure(0, weight=1)
        ttk.Label(self.ros2_panel_frame, text=f"rospkg read-only source: {ROS_PACKAGE_DIR}", anchor="w").grid(
            row=0, column=0, columnspan=3, sticky="ew"
        )
        fcu_row = ttk.Frame(self.ros2_panel_frame)
        fcu_row.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(3, 0))
        fcu_row.columnconfigure(1, weight=1)
        ttk.Label(fcu_row, text="fcu_url").grid(row=0, column=0, sticky="w", padx=(0, 4))
        ttk.Entry(fcu_row, textvariable=self.ros_pkg_fcu_url_var).grid(row=0, column=1, sticky="ew")
        ros2_buttons = ttk.Frame(self.ros2_panel_frame)
        ros2_buttons.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(4, 0))
        ttk.Button(ros2_buttons, text="Build pkg (Ubuntu)", command=self._build_ros_pkg).pack(side=tk.LEFT)
        self.mavros_toggle_button = ttk.Button(
            ros2_buttons,
            text="MAVROS ON",
            style="Danger.TButton",
            command=self._toggle_ros_pkg_stack,
        )
        self.mavros_toggle_button.pack(side=tk.LEFT, padx=(4, 0))
        self.rviz_toggle_button = ttk.Button(
            ros2_buttons,
            text="RViz ON",
            style="Danger.TButton",
            command=self._toggle_rviz,
        )
        self.rviz_toggle_button.pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(self.ros2_panel_frame, textvariable=self.ros_pkg_status_var, anchor="w", style="Status.TLabel").grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=(3, 0)
        )
        ttk.Label(self.ros2_panel_frame, textvariable=self.rviz_status_var, anchor="w", style="Status.TLabel").grid(
            row=4, column=0, columnspan=3, sticky="ew"
        )
        self.ros2_panel_frame.grid_remove()

        arm_row = ttk.Frame(control_box)
        arm_row.grid(row=4, column=0, sticky="ew", pady=(0, 4))
        ttk.Button(arm_row, text="Arm", style="Success.TButton", command=lambda: self.node.arm(True)).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(arm_row, text="Disarm", style="Danger.TButton", command=lambda: self.node.arm(False)).pack(
            side=tk.LEFT
        )

        mode_row = ttk.LabelFrame(control_box, text="Modes", padding=INNER_PADDING)
        mode_row.grid(row=5, column=0, sticky="ew", pady=(0, 4))
        for idx, mode in enumerate(self.MODE_BUTTONS):
            ttk.Button(mode_row, text=mode, command=lambda m=mode: self.node.set_mode(m)).grid(
                row=idx // 3, column=idx % 3, sticky="ew", padx=3, pady=3
            )
        for col in range(3):
            mode_row.columnconfigure(col, weight=1)

        replay_box = ttk.LabelFrame(control_box, text="RC Override Replay", padding=INNER_PADDING)
        replay_box.grid(row=6, column=0, sticky="ew", pady=(0, 4))
        replay_box.columnconfigure(0, weight=1)

        replay_path_row = ttk.Frame(replay_box)
        replay_path_row.grid(row=0, column=0, sticky="ew")
        replay_path_row.columnconfigure(0, weight=1)
        ttk.Entry(replay_path_row, textvariable=self.rc_replay_path_var).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(replay_path_row, text="Browse", command=self._browse_rc_replay_bag).grid(row=0, column=1)

        timeline_row = ttk.Frame(replay_box)
        timeline_row.grid(row=1, column=0, sticky="ew", pady=(4, 0))
        timeline_row.columnconfigure(0, weight=1)
        self.rc_replay_slider = ttk.Scale(
            timeline_row,
            from_=0.0,
            to=1.0,
            orient=tk.HORIZONTAL,
            variable=self.rc_replay_position_var,
            command=self._on_rc_replay_slider_changed,
        )
        self.rc_replay_slider.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.rc_replay_slider.state(["disabled"])
        self.rc_replay_slider.bind("<Button-1>", self._on_rc_replay_slider_press)
        self.rc_replay_slider.bind("<B1-Motion>", self._on_rc_replay_slider_motion)
        self.rc_replay_slider.bind("<ButtonRelease-1>", self._on_rc_replay_slider_release)
        ttk.Label(timeline_row, textvariable=self.rc_replay_time_var, width=15, anchor="e").grid(
            row=0, column=1, sticky="e"
        )

        replay_controls = ttk.Frame(replay_box)
        replay_controls.grid(row=2, column=0, sticky="ew", pady=(3, 0))
        ttk.Button(replay_controls, text="Load", command=self._load_rc_replay).pack(side=tk.LEFT)
        self.rc_replay_play_button = ttk.Button(replay_controls, text="Play", command=self._start_rc_replay)
        self.rc_replay_play_button.pack(side=tk.LEFT, padx=(4, 0))
        self.rc_replay_pause_button = ttk.Button(
            replay_controls,
            text="Pause",
            command=self._toggle_rc_replay_pause,
        )
        self.rc_replay_pause_button.pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(replay_controls, text="Stop", style="Danger.TButton", command=self._stop_rc_replay).pack(
            side=tk.LEFT, padx=(4, 0)
        )
        ttk.Label(replay_controls, text="rate").pack(side=tk.LEFT, padx=(10, 2))
        ttk.Entry(replay_controls, width=4, textvariable=self.rc_replay_rate_var).pack(side=tk.LEFT)

        ttk.Label(replay_box, textvariable=self.rc_replay_status_var, anchor="w", style="Status.TLabel").grid(
            row=3, column=0, sticky="ew", pady=(2, 0)
        )

        physics_row = ttk.LabelFrame(control_box, text="Sim Param Tuning", padding=INNER_PADDING)
        physics_row.grid(row=7, column=0, sticky="ew", pady=(0, 4))
        physics_row.columnconfigure(0, weight=1)
        ttk.Label(
            physics_row,
            textvariable=self.physics_status_var,
            anchor="w",
            style="Status.TLabel",
        ).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.physics_toggle_button = ttk.Button(
            physics_row,
            text="Open physics params",
            style="Accent.TButton",
            command=self._show_physics_window,
        )
        self.physics_toggle_button.grid(row=0, column=1, sticky="e")
        self._load_physics_params_into_fields(silent=True)

        autotune_toggle_row = ttk.Frame(control_box)
        autotune_toggle_row.grid(row=8, column=0, sticky="ew", pady=(0, 4))
        autotune_toggle_row.columnconfigure(0, weight=1)
        ttk.Label(autotune_toggle_row, textvariable=self.autotune_status_var, anchor="w", style="Status.TLabel").grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        self.autotune_toggle_button = ttk.Button(
            autotune_toggle_row,
            text="Show auto tune",
            command=self._toggle_autotune_panel,
        )
        self.autotune_toggle_button.grid(row=0, column=1, sticky="e")

        self.autotune_frame = ttk.LabelFrame(control_box, text="Auto Tune", padding=INNER_PADDING)
        self.autotune_frame.grid(row=9, column=0, sticky="ew", pady=(0, 4))
        self.autotune_frame.columnconfigure(1, weight=1)
        ttk.Label(self.autotune_frame, text="bag").grid(row=0, column=0, sticky="w")
        ttk.Entry(self.autotune_frame, textvariable=self.autotune_bag_var).grid(
            row=0, column=1, columnspan=5, sticky="ew", padx=(4, 4)
        )
        ttk.Button(self.autotune_frame, text="Browse", command=self._browse_autotune_bag).grid(
            row=0, column=6, sticky="e"
        )
        ttk.Label(self.autotune_frame, text="start").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(self.autotune_frame, width=7, textvariable=self.autotune_start_var).grid(
            row=1, column=1, sticky="ew", padx=(4, 6), pady=(4, 0)
        )
        ttk.Label(self.autotune_frame, text="dur").grid(row=1, column=2, sticky="w", pady=(4, 0))
        ttk.Entry(self.autotune_frame, width=7, textvariable=self.autotune_duration_var).grid(
            row=1, column=3, sticky="ew", padx=(4, 6), pady=(4, 0)
        )
        ttk.Label(self.autotune_frame, text="n").grid(row=1, column=4, sticky="w", pady=(4, 0))
        ttk.Entry(self.autotune_frame, width=5, textvariable=self.autotune_candidates_var).grid(
            row=1, column=5, sticky="ew", padx=(4, 0), pady=(4, 0)
        )
        ttk.Label(self.autotune_frame, text="servo").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(self.autotune_frame, width=7, textvariable=self.autotune_servo_scale_var).grid(
            row=2, column=1, sticky="ew", padx=(4, 6), pady=(4, 0)
        )
        ttk.Label(self.autotune_frame, text="mode").grid(row=2, column=2, sticky="w", pady=(4, 0))
        ttk.Entry(self.autotune_frame, width=13, textvariable=self.autotune_mode_var).grid(
            row=2, column=3, sticky="ew", padx=(4, 6), pady=(4, 0)
        )
        ttk.Label(self.autotune_frame, text="set").grid(row=2, column=4, sticky="w", pady=(4, 0))
        ttk.Entry(self.autotune_frame, width=10, textvariable=self.autotune_candidate_set_var).grid(
            row=2, column=5, sticky="ew", padx=(4, 0), pady=(4, 0)
        )
        autotune_buttons = ttk.Frame(self.autotune_frame)
        autotune_buttons.grid(row=3, column=0, columnspan=7, sticky="ew", pady=(5, 0))
        ttk.Button(autotune_buttons, text="Start", style="Accent.TButton", command=self._start_autotune).pack(
            side=tk.LEFT
        )
        ttk.Button(autotune_buttons, text="Stop", style="Danger.TButton", command=self._stop_autotune).pack(
            side=tk.LEFT, padx=(4, 0)
        )
        ttk.Button(autotune_buttons, text="Open output", command=self._open_autotune_output).pack(
            side=tk.LEFT, padx=(4, 0)
        )
        ttk.Checkbutton(
            autotune_buttons,
            text="apply best",
            variable=self.autotune_apply_best_var,
        ).pack(side=tk.LEFT, padx=(10, 0))
        self.autotune_frame.grid_remove()

        rc_box = ttk.LabelFrame(control_box, text="RC Override Joysticks", padding=INNER_PADDING)
        rc_box.grid(row=10, column=0, sticky="ew", pady=(0, 4))
        rc_box.columnconfigure(0, weight=1)

        rc_header = ttk.Frame(rc_box)
        rc_header.grid(row=0, column=0, sticky="ew", pady=(0, 3))
        ttk.Checkbutton(
            rc_header,
            text="Enable RC override",
            variable=self.rc_override_enabled,
            command=self._on_rc_override_toggle,
        ).pack(side=tk.LEFT)
        ttk.Button(rc_header, text="Center", command=self._center_rc_sticks).pack(side=tk.RIGHT)
        ttk.Button(
            rc_header,
            text="Release RC",
            style="Danger.TButton",
            command=self._release_rc_override,
        ).pack(side=tk.RIGHT, padx=(0, 6))

        stick_row = ttk.Frame(rc_box)
        stick_row.grid(row=1, column=0, sticky="ew")
        stick_row.columnconfigure(0, weight=1)
        stick_row.columnconfigure(1, weight=1)

        self.left_stick = VirtualJoystick(
            stick_row,
            title="Left Stick",
            x_var=self.rc_yaw_var,
            y_var=self.rc_heave_var,
            x_label="yaw",
            y_label="heave",
        )
        self.left_stick.grid(row=0, column=0, sticky="nsew", padx=(0, 3))

        self.right_stick = VirtualJoystick(
            stick_row,
            title="Right Stick",
            x_var=self.rc_lateral_var,
            y_var=self.rc_forward_var,
            x_label="lateral",
            y_label="forward",
        )
        self.right_stick.grid(row=0, column=1, sticky="nsew", padx=(3, 0))

        details_row = ttk.Frame(control_box)
        details_row.grid(row=12, column=0, sticky="ew", pady=(4, 0))
        self.control_details_button = ttk.Button(
            details_row,
            text="Show control details",
            command=self._toggle_control_details,
        )
        self.control_details_button.pack(side=tk.RIGHT)

        self.control_details_frame = ttk.LabelFrame(control_box, text="Control Details", padding=INNER_PADDING)
        self.control_details_frame.grid(row=13, column=0, sticky="ew", pady=(3, 0))
        self.control_details_frame.columnconfigure(0, weight=1)
        ttk.Label(self.control_details_frame, textvariable=self.control_var, anchor="w").grid(
            row=0, column=0, sticky="ew", pady=1
        )
        ttk.Label(self.control_details_frame, textvariable=self.rc_override_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=1
        )
        self.control_details_frame.grid_remove()
