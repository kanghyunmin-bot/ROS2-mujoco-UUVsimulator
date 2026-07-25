"""Window shell and scroll-frame helpers for GUI physics tuning."""

from __future__ import annotations

from typing import Any

import tkinter as tk
from tkinter import ttk

from .config import PHYSICS_PROFILE_NAME, PHYSICS_PROFILE_PATH


def restore_existing_physics_window(owner: Any) -> bool:
    if owner.physics_window is None or not owner.physics_window.winfo_exists():
        return False
    owner._load_physics_params_into_fields(silent=True)
    owner.physics_window.deiconify()
    owner.physics_window.lift()
    return True


def create_physics_window_shell(owner: Any) -> tuple[ttk.Frame, ttk.Frame]:
    win = tk.Toplevel(owner.root)
    win.title("UUV Sim Param Tuning")
    win.geometry("980x640")
    win.minsize(860, 500)
    win.protocol("WM_DELETE_WINDOW", owner._close_physics_window)
    owner.physics_window = win

    outer = _build_outer_frame(win)
    _build_header(owner, outer)
    _build_status_label(owner, outer)
    grid = _build_scroll_grid(owner, outer)
    return outer, grid


def _build_outer_frame(win: tk.Toplevel) -> ttk.Frame:
    outer = ttk.Frame(win, padding=8)
    outer.pack(fill=tk.BOTH, expand=True)
    outer.columnconfigure(0, weight=1)
    outer.rowconfigure(2, weight=1)
    return outer


def _build_header(owner: Any, outer: ttk.Frame) -> None:
    header = ttk.Frame(outer)
    header.grid(row=0, column=0, sticky="ew")
    header.columnconfigure(0, weight=1)
    ttk.Label(
        header,
        text=f"profile: {PHYSICS_PROFILE_NAME}   file: {PHYSICS_PROFILE_PATH}",
        anchor="w",
    ).grid(row=0, column=0, sticky="ew")
    ttk.Button(header, text="Reload", command=owner._load_physics_params_into_fields).grid(
        row=0,
        column=1,
        padx=(8, 0),
    )
    ttk.Button(header, text="Apply", command=owner._apply_physics_params).grid(row=0, column=2, padx=(4, 0))
    ttk.Button(header, text="Apply + Restart", command=lambda: owner._apply_physics_params(restart=True)).grid(
        row=0,
        column=3,
        padx=(4, 0),
    )


def _build_status_label(owner: Any, outer: ttk.Frame) -> None:
    ttk.Label(outer, textvariable=owner.physics_status_var, anchor="w").grid(
        row=1,
        column=0,
        sticky="ew",
        pady=(6, 6),
    )


def _build_scroll_grid(owner: Any, outer: ttk.Frame) -> ttk.Frame:
    body = ttk.Frame(outer)
    body.grid(row=2, column=0, sticky="nsew")
    body.columnconfigure(0, weight=1)
    body.rowconfigure(0, weight=1)

    canvas = tk.Canvas(body, highlightthickness=0)
    scroll_y = ttk.Scrollbar(body, orient=tk.VERTICAL, command=canvas.yview)
    canvas.configure(yscrollcommand=scroll_y.set)
    canvas.grid(row=0, column=0, sticky="nsew")
    scroll_y.grid(row=0, column=1, sticky="ns")
    owner.physics_canvas = canvas

    grid = ttk.Frame(canvas, padding=(0, 0, 6, 0))
    owner.physics_scroll_frame = grid
    window_id = canvas.create_window((0, 0), window=grid, anchor="nw")
    grid.columnconfigure(1, weight=1)
    grid.columnconfigure(3, weight=2)
    grid.columnconfigure(4, weight=1)

    grid.bind("<Configure>", lambda _event=None: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window_id, width=event.width))
    return grid


__all__ = ["create_physics_window_shell", "restore_existing_physics_window"]
