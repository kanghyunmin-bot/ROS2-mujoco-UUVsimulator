"""Reusable Tk widgets for the MuJoCo UUV GUI."""

from __future__ import annotations

import tkinter as tk
from typing import Callable

from .virtual_joystick_runtime import initialize_virtual_joystick


class VirtualJoystick(tk.Frame):
    def __init__(
        self,
        parent,
        *,
        title: str,
        x_var: tk.DoubleVar,
        y_var: tk.DoubleVar,
        x_label: str,
        y_label: str,
        on_change: Callable[[], None] | None = None,
    ) -> None:
        super().__init__(
            parent,
            bg="#111827",
            highlightbackground="#334155",
            highlightcolor="#334155",
            highlightthickness=1,
            bd=0,
        )
        initialize_virtual_joystick(
            self,
            title=title,
            x_var=x_var,
            y_var=y_var,
            x_label=x_label,
            y_label=y_label,
            on_change=on_change,
        )


__all__ = ["VirtualJoystick"]
