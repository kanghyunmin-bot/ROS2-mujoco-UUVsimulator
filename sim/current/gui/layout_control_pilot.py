"""Pilot control layout builders."""

from __future__ import annotations

from .config import INNER_PADDING
from .runtime import tk, ttk
from .widgets import VirtualJoystick


def build_pilot_section(owner, control_box) -> None:
    rc_box = ttk.LabelFrame(control_box, text="Pilot Control", padding=INNER_PADDING)
    rc_box.grid(row=10, column=0, sticky="ew", pady=(0, 4))
    rc_box.columnconfigure(0, weight=1)
    build_pilot_header(owner, rc_box)
    build_sticks(owner, rc_box)
    build_control_details(owner, control_box)


def build_pilot_header(owner, rc_box) -> None:
    rc_header = ttk.Frame(rc_box, style="PilotHeader.TFrame")
    rc_header.grid(row=0, column=0, sticky="ew", pady=(0, 5))
    rc_header.columnconfigure(1, weight=1)
    ttk.Checkbutton(
        rc_header,
        text="Pilot input",
        variable=owner.rc_override_enabled,
        command=owner._on_rc_override_toggle,
        style="Pilot.TCheckbutton",
    ).grid(row=0, column=0, sticky="w")
    ttk.Label(
        rc_header,
        textvariable=owner.control_summary_var,
        style="PilotHint.TLabel",
    ).grid(row=0, column=1, sticky="w", padx=(8, 0))

    button_row = ttk.Frame(rc_header, style="PilotHeader.TFrame")
    button_row.grid(row=0, column=2, sticky="e")
    ttk.Button(
        button_row,
        text="Center sticks",
        style="Compact.TButton",
        command=owner._center_rc_sticks,
    ).pack(side=tk.LEFT)
    ttk.Button(
        button_row,
        text="Release input",
        style="CompactDanger.TButton",
        command=owner._release_rc_override,
    ).pack(side=tk.LEFT, padx=(5, 0))


def build_sticks(owner, rc_box) -> None:
    stick_row = ttk.Frame(rc_box)
    stick_row.grid(row=1, column=0, sticky="ew")
    stick_row.columnconfigure(0, weight=1)
    stick_row.columnconfigure(1, weight=1)

    owner.left_stick = VirtualJoystick(
        stick_row,
        title="Left Stick",
        x_var=owner.rc_yaw_var,
        y_var=owner.rc_heave_var,
        x_label="yaw",
        y_label="heave",
        on_change=owner._on_rc_stick_changed,
    )
    owner.left_stick.grid(row=0, column=0, sticky="nsew", padx=(0, 3))

    owner.right_stick = VirtualJoystick(
        stick_row,
        title="Right Stick",
        x_var=owner.rc_lateral_var,
        y_var=owner.rc_forward_var,
        x_label="lateral",
        y_label="forward",
        on_change=owner._on_rc_stick_changed,
    )
    owner.right_stick.grid(row=0, column=1, sticky="nsew", padx=(3, 0))


def build_control_details(owner, control_box) -> None:
    details_row = ttk.Frame(control_box)
    details_row.grid(row=12, column=0, sticky="ew", pady=(4, 0))
    owner.control_details_button = ttk.Button(
        details_row,
        text="Details",
        style="Compact.TButton",
        command=owner._toggle_control_details,
    )
    owner.control_details_button.pack(side=tk.RIGHT)

    owner.control_details_frame = ttk.LabelFrame(control_box, text="Control Details", padding=INNER_PADDING)
    owner.control_details_frame.grid(row=13, column=0, sticky="ew", pady=(3, 0))
    owner.control_details_frame.columnconfigure(0, weight=1)
    ttk.Label(owner.control_details_frame, textvariable=owner.control_var, anchor="w").grid(
        row=0, column=0, sticky="ew", pady=1
    )
    ttk.Label(owner.control_details_frame, textvariable=owner.rc_override_var, anchor="w").grid(
        row=1, column=0, sticky="ew", pady=1
    )
    owner.control_details_frame.grid_remove()
