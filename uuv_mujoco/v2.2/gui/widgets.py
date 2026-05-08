"""Reusable Tk widgets for the MuJoCo UUV GUI."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .config import AXIS_DEADBAND, INNER_PADDING, JOYSTICK_CANVAS_SIZE
from .helpers import clamp_axis

class VirtualJoystick(ttk.LabelFrame):
    def __init__(
        self,
        parent,
        *,
        title: str,
        x_var: tk.DoubleVar,
        y_var: tk.DoubleVar,
        x_label: str,
        y_label: str,
    ) -> None:
        super().__init__(parent, text=title, padding=INNER_PADDING)
        self.x_var = x_var
        self.y_var = y_var
        self.x_label = x_label
        self.y_label = y_label

        self.canvas = tk.Canvas(
            self,
            width=JOYSTICK_CANVAS_SIZE,
            height=JOYSTICK_CANVAS_SIZE,
            bg="#0b1220",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.value_label = ttk.Label(self, anchor="center")
        self.value_label.pack(fill=tk.X, pady=(3, 0))

        self.canvas.bind("<Configure>", self._redraw)
        self.canvas.bind("<Button-1>", self._on_drag)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Double-Button-1>", self._on_release)

        self.x_var.trace_add("write", self._redraw)
        self.y_var.trace_add("write", self._redraw)
        self._redraw()

    def _geometry(self) -> tuple[int, int, float, float, float]:
        width = max(self.canvas.winfo_width(), 130)
        height = max(self.canvas.winfo_height(), 130)
        cx = width / 2.0
        cy = height / 2.0
        radius = min(width, height) * 0.34
        return width, height, cx, cy, radius

    def _set_axes(self, x: float, y: float) -> None:
        if abs(x) < AXIS_DEADBAND:
            x = 0.0
        if abs(y) < AXIS_DEADBAND:
            y = 0.0
        self.x_var.set(clamp_axis(x))
        self.y_var.set(clamp_axis(y))

    def _on_drag(self, event) -> None:
        _, _, cx, cy, radius = self._geometry()
        x = (event.x - cx) / radius
        y = (cy - event.y) / radius
        self._set_axes(x, y)

    def _on_release(self, _event=None) -> None:
        self._set_axes(0.0, 0.0)

    def _redraw(self, *_args) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width, height, cx, cy, radius = self._geometry()
        x = clamp_axis(self.x_var.get())
        y = clamp_axis(self.y_var.get())

        canvas.create_rectangle(10, 10, width - 10, height - 10, outline="#334155", width=2)
        canvas.create_line(cx, 20, cx, height - 20, fill="#334155", width=2)
        canvas.create_line(20, cy, width - 20, cy, fill="#334155", width=2)
        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#475569", width=2)

        knob_x = cx + x * radius
        knob_y = cy - y * radius
        canvas.create_line(cx, cy, knob_x, knob_y, fill="#38bdf8", width=3)
        knob_radius = max(10, min(width, height) * 0.055)
        canvas.create_oval(
            knob_x - knob_radius,
            knob_y - knob_radius,
            knob_x + knob_radius,
            knob_y + knob_radius,
            fill="#0ea5e9",
            outline="#e0f2fe",
            width=2,
        )

        canvas.create_text(cx, 18, text=self.y_label, fill="#cbd5e1")
        canvas.create_text(width - 18, cy - 10, text=self.x_label, fill="#cbd5e1", anchor="e")
        canvas.create_text(18, cy - 10, text=f"-{self.x_label}", fill="#64748b", anchor="w")
        canvas.create_text(cx, height - 18, text=f"-{self.y_label}", fill="#64748b")

        self.value_label.config(text=f"{self.y_label}={y:+.2f}  {self.x_label}={x:+.2f}")
