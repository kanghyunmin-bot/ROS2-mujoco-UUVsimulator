"""Simulation stack controls for the GUI control panel."""

from __future__ import annotations

from .config import INNER_PADDING
from .runtime import tk, ttk


def build_telemetry_toggle(owner, control_box) -> None:
    telemetry_row = ttk.Frame(control_box)
    telemetry_row.grid(row=0, column=0, sticky="ew", pady=(0, 4))
    telemetry_row.columnconfigure(0, weight=1)
    ttk.Label(telemetry_row, text="Telemetry panel").grid(row=0, column=0, sticky="w")
    owner.telemetry_toggle_button = ttk.Button(
        telemetry_row,
        text="Hide telemetry",
        command=owner._toggle_telemetry_panel,
    )
    owner.telemetry_toggle_button.grid(row=0, column=1, sticky="e")


def build_sim_stack_controls(owner, control_box) -> None:
    stack_row = ttk.LabelFrame(control_box, text="Simulation Stack", padding=INNER_PADDING)
    stack_row.grid(row=1, column=0, sticky="ew", pady=(0, 4))
    stack_row.columnconfigure(0, weight=1)
    stack_buttons = ttk.Frame(stack_row)
    stack_buttons.grid(row=0, column=0, sticky="ew")
    owner.sim_stack_start_button = ttk.Button(
        stack_buttons,
        text="Start SITL/MuJoCo",
        style="Success.TButton",
        command=owner._start_sim_stack,
    )
    owner.sim_stack_start_button.pack(side=tk.LEFT)
    owner.sim_stack_stop_button = ttk.Button(
        stack_buttons,
        text="Stop/Reset",
        style="Danger.TButton",
        command=owner._stop_sim_stack,
    )
    owner.sim_stack_stop_button.pack(side=tk.LEFT, padx=(4, 0))
    ttk.Button(
        stack_buttons,
        text="Ping360 panel",
        style="Info.TButton",
        command=owner._toggle_ping360_window,
    ).pack(side=tk.LEFT, padx=(10, 0))
    ttk.Label(stack_row, textvariable=owner.sim_stack_status_var, anchor="w", style="Status.TLabel").grid(
        row=1, column=0, sticky="ew", pady=(2, 0)
    )
