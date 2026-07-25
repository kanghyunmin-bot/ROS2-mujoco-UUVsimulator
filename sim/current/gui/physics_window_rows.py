"""Grid rows for the GUI physics tuning window."""

from __future__ import annotations

from typing import Any

from tkinter import ttk

from .config import PHYSICS_PARAM_SPECS
from .physics_param_io import _physics_current_mode_status, _physics_param_inactive_in_current


def build_physics_grid(owner: Any, grid: ttk.Frame) -> None:
    _build_physics_header_row(grid)
    for row_idx, spec in enumerate(PHYSICS_PARAM_SPECS, start=1):
        _build_physics_param_row(owner, grid, row_idx, spec)


def _build_physics_header_row(grid: ttk.Frame) -> None:
    headers = [
        ("parameter", 0, {}),
        ("value", 1, {}),
        ("key", 2, {"padx": (8, 8)}),
        ("what it changes", 3, {}),
        ("current mode", 4, {"padx": (8, 0)}),
    ]
    for text, column, grid_kwargs in headers:
        ttk.Label(grid, text=text, anchor="w").grid(
            row=0,
            column=column,
            sticky="ew",
            pady=(0, 4),
            **grid_kwargs,
        )


def _build_physics_param_row(owner: Any, grid: ttk.Frame, row_idx: int, spec: dict[str, Any]) -> None:
    key = str(spec["key"])
    inactive = _physics_param_inactive_in_current(key)
    foreground = "#94a3b8" if inactive else "#0f172a"
    ttk.Label(grid, text=str(spec["label"]), width=25, anchor="w", foreground=foreground).grid(
        row=row_idx,
        column=0,
        sticky="w",
        pady=2,
    )
    entry_state = "readonly" if inactive else "normal"
    ttk.Entry(grid, textvariable=owner.physics_param_vars[key], width=24, state=entry_state).grid(
        row=row_idx,
        column=1,
        sticky="ew",
        padx=(4, 8),
        pady=2,
    )
    ttk.Label(grid, text=key, anchor="w", foreground="#64748b").grid(
        row=row_idx,
        column=2,
        sticky="w",
        padx=(0, 8),
        pady=2,
    )
    ttk.Label(
        grid,
        text=str(spec["description"]),
        anchor="w",
        foreground="#475569",
        wraplength=300,
    ).grid(row=row_idx, column=3, sticky="ew", pady=2)
    ttk.Label(
        grid,
        text=_physics_current_mode_status(key),
        anchor="w",
        foreground="#b45309" if inactive else "#166534",
        wraplength=180,
    ).grid(row=row_idx, column=4, sticky="ew", padx=(8, 0), pady=2)


def build_physics_footer(owner: Any, outer: ttk.Frame) -> None:
    footer = ttk.Frame(outer)
    footer.grid(row=3, column=0, sticky="ew", pady=(8, 0))
    footer.columnconfigure(0, weight=1)
    ttk.Label(
        footer,
        text=(
            "Only current-mode active rows are applied. Inactive rows are read-only because "
            "running MuJoCo current mode does not consume those parameters."
        ),
        anchor="w",
        foreground="#64748b",
    ).grid(row=0, column=0, sticky="ew")
    ttk.Button(footer, text="Close", command=owner._close_physics_window).grid(row=0, column=1, padx=(8, 0))


__all__ = ["build_physics_footer", "build_physics_grid"]
