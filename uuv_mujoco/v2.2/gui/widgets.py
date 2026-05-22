"""Reusable Tk widgets for the MuJoCo UUV GUI."""

from __future__ import annotations

import math
import tkinter as tk
from typing import Callable

from .config import AXIS_DEADBAND, INNER_PADDING, JOYSTICK_CANVAS_SIZE
from .helpers import clamp_axis

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
        self.x_var = x_var
        self.y_var = y_var
        self.x_label = x_label
        self.y_label = y_label
        self.on_change = on_change
        self.title = title

        self.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            self,
            width=JOYSTICK_CANVAS_SIZE,
            height=JOYSTICK_CANVAS_SIZE + 18,
            bg="#111827",
            highlightthickness=0,
            bd=0,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.rowconfigure(0, weight=1)

        self.canvas.bind("<Configure>", self._redraw)
        self.canvas.bind("<Button-1>", self._on_drag)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Double-Button-1>", self._on_release)

        self.x_var.trace_add("write", self._redraw)
        self.y_var.trace_add("write", self._redraw)
        self._redraw()

    def _geometry(self) -> tuple[int, int, float, float, float]:
        width = max(self.canvas.winfo_width(), JOYSTICK_CANVAS_SIZE)
        height = max(self.canvas.winfo_height(), JOYSTICK_CANVAS_SIZE + 18)
        cx = width / 2.0
        cy = height / 2.0 + 10.0
        radius = min(width, height - 18) * 0.29
        return width, height, cx, cy, radius

    def _set_axes(self, x: float, y: float) -> None:
        if abs(x) < AXIS_DEADBAND:
            x = 0.0
        if abs(y) < AXIS_DEADBAND:
            y = 0.0
        self.x_var.set(clamp_axis(x))
        self.y_var.set(clamp_axis(y))
        if self.on_change is not None:
            self.on_change()

    def _on_drag(self, event) -> None:
        _, _, cx, cy, radius = self._geometry()
        x = (event.x - cx) / radius
        y = (cy - event.y) / radius
        magnitude = math.hypot(x, y)
        if magnitude > 1.0:
            x /= magnitude
            y /= magnitude
        self._set_axes(x, y)

    def _on_release(self, _event=None) -> None:
        self._set_axes(0.0, 0.0)

    def _redraw(self, *_args) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width, height, cx, cy, radius = self._geometry()
        x = clamp_axis(self.x_var.get())
        y = clamp_axis(self.y_var.get())
        magnitude = math.hypot(x, y)
        if magnitude > 1.0:
            x /= magnitude
            y /= magnitude
            magnitude = 1.0

        canvas.create_rectangle(0, 0, width, height, fill="#111827", outline="")
        canvas.create_text(
            12,
            12,
            text=self.title,
            fill="#e5e7eb",
            anchor="nw",
            font=("TkDefaultFont", 10, "bold"),
        )
        canvas.create_text(
            width - 12,
            12,
            text=f"{self.y_label} {y:+.2f}   {self.x_label} {x:+.2f}",
            fill="#94a3b8",
            anchor="ne",
            font=("TkDefaultFont", 8),
        )

        base_r = radius * 1.42
        gate_r = radius * 1.02
        canvas.create_oval(
            cx - base_r + 3,
            cy - base_r + 6,
            cx + base_r + 3,
            cy + base_r + 6,
            fill="#020617",
            outline="",
        )
        canvas.create_oval(
            cx - base_r,
            cy - base_r,
            cx + base_r,
            cy + base_r,
            fill="#1f2937",
            outline="#475569",
            width=2,
        )
        for idx, color in enumerate(("#182235", "#141d2e", "#101827", "#0b1322")):
            r = base_r * (0.86 - idx * 0.13)
            canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color, outline="")
        canvas.create_oval(
            cx - gate_r,
            cy - gate_r,
            cx + gate_r,
            cy + gate_r,
            outline="#64748b",
            width=2,
        )
        canvas.create_oval(
            cx - gate_r * 0.55,
            cy - gate_r * 0.55,
            cx + gate_r * 0.55,
            cy + gate_r * 0.55,
            outline="#263549",
            width=1,
        )
        notch_r = base_r * 0.93
        notch_len = 8
        for angle_deg in (0, 90, 180, 270):
            angle = math.radians(angle_deg)
            x0 = cx + math.cos(angle) * (notch_r - notch_len)
            y0 = cy + math.sin(angle) * (notch_r - notch_len)
            x1 = cx + math.cos(angle) * notch_r
            y1 = cy + math.sin(angle) * notch_r
            canvas.create_line(x0, y0, x1, y1, fill="#94a3b8", width=2)

        knob_x = cx + x * radius
        knob_y = cy - y * radius
        shaft_r = radius * 0.42
        canvas.create_oval(
            knob_x - shaft_r + 2,
            knob_y - shaft_r + 4,
            knob_x + shaft_r + 2,
            knob_y + shaft_r + 4,
            fill="#020617",
            outline="",
        )
        canvas.create_line(cx, cy, knob_x, knob_y, fill="#0f7490", width=4 if magnitude else 2)
        canvas.create_oval(
            knob_x - shaft_r,
            knob_y - shaft_r,
            knob_x + shaft_r,
            knob_y + shaft_r,
            fill="#0f172a",
            outline="#38bdf8" if magnitude > AXIS_DEADBAND else "#64748b",
            width=2,
        )
        for idx, color in enumerate(("#1e293b", "#27364a", "#334155")):
            r = shaft_r * (0.72 - idx * 0.17)
            canvas.create_oval(knob_x - r, knob_y - r, knob_x + r, knob_y + r, fill=color, outline="")
        cap_r = shaft_r * 0.28
        canvas.create_oval(
            knob_x - cap_r,
            knob_y - cap_r,
            knob_x + cap_r,
            knob_y + cap_r,
            fill="#38bdf8" if magnitude > AXIS_DEADBAND else "#94a3b8",
            outline="",
        )

        label_r = base_r * 1.10
        canvas.create_text(cx, cy - label_r, text=self.y_label, fill="#cbd5e1", font=("TkDefaultFont", 9, "bold"))
        canvas.create_text(cx, cy + label_r, text=f"-{self.y_label}", fill="#64748b", font=("TkDefaultFont", 9))
        canvas.create_text(
            cx + label_r,
            cy,
            text=self.x_label,
            fill="#cbd5e1",
            anchor="w",
            font=("TkDefaultFont", 9, "bold"),
        )
        canvas.create_text(cx - label_r, cy, text=f"-{self.x_label}", fill="#64748b", anchor="e", font=("TkDefaultFont", 9))
