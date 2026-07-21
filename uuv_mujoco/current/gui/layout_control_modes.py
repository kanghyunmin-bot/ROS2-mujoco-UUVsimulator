"""Arm and flight-mode controls for the GUI control panel."""

from __future__ import annotations

from .config import INNER_PADDING
from .runtime import tk, ttk


def build_arm_and_modes(owner, control_box) -> None:
    arm_row = ttk.Frame(control_box)
    arm_row.grid(row=4, column=0, sticky="ew", pady=(0, 4))
    ttk.Button(arm_row, text="Arm", style="Success.TButton", command=lambda: owner.node.arm(True)).pack(
        side=tk.LEFT, padx=(0, 6)
    )
    ttk.Button(arm_row, text="Disarm", style="Danger.TButton", command=lambda: owner.node.arm(False)).pack(
        side=tk.LEFT
    )
    owner.command_ready_label = ttk.Label(
        arm_row,
        textvariable=owner.command_ready_var,
        style="NotReady.TLabel",
        anchor="center",
    )
    owner.command_ready_label.pack(side=tk.LEFT, padx=(8, 0), fill=tk.X, expand=True)

    mode_row = ttk.LabelFrame(control_box, text="Modes", padding=INNER_PADDING)
    mode_row.grid(row=5, column=0, sticky="ew", pady=(0, 4))
    for idx, mode in enumerate(owner.MODE_BUTTONS):
        ttk.Button(mode_row, text=mode, command=lambda m=mode: owner.node.set_mode(m)).grid(
            row=idx // 3, column=idx % 3, sticky="ew", padx=3, pady=3
        )
    for col in range(3):
        mode_row.columnconfigure(col, weight=1)
