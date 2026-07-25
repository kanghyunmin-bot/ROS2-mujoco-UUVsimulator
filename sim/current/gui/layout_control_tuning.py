"""Physics tuning control layout builders."""

from __future__ import annotations

from .config import INNER_PADDING
from .runtime import ttk


def build_tuning_sections(owner, parent, *, start_row: int = 7) -> None:
    build_physics_section(owner, parent, row=start_row)
    build_buoy_layout_section(owner, parent, row=start_row + 1)


def build_physics_section(owner, parent, *, row: int = 7) -> None:
    physics_row = ttk.LabelFrame(parent, text="Sim Param Tuning", padding=INNER_PADDING)
    physics_row.grid(row=row, column=0, sticky="ew", pady=(0, 4))
    physics_row.columnconfigure(0, weight=1)
    ttk.Label(
        physics_row,
        textvariable=owner.physics_status_var,
        anchor="w",
        style="Status.TLabel",
    ).grid(row=0, column=0, sticky="ew", padx=(0, 6))
    owner.physics_toggle_button = ttk.Button(
        physics_row,
        text="Open physics params",
        style="Accent.TButton",
        command=owner._show_physics_window,
    )
    owner.physics_toggle_button.grid(row=0, column=1, sticky="e")
    owner._load_physics_params_into_fields(silent=True)


def build_buoy_layout_section(owner, parent, *, row: int = 8) -> None:
    layout_row = ttk.LabelFrame(parent, text="Course Layout", padding=INNER_PADDING)
    layout_row.grid(row=row, column=0, sticky="ew", pady=(0, 4))
    layout_row.columnconfigure(0, weight=1)
    ttk.Label(
        layout_row,
        textvariable=owner.buoy_layout_status_var,
        anchor="w",
        style="Status.TLabel",
    ).grid(row=0, column=0, sticky="ew", padx=(0, 6))
    owner.buoy_layout_button = ttk.Button(
        layout_row,
        text="Open course XY layout",
        style="Info.TButton",
        command=lambda: owner.root.after(1, owner._show_buoy_layout_window),
    )
    owner.buoy_layout_button.grid(row=0, column=1, sticky="e")
