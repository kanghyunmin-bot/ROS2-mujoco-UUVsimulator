"""Canvas renderer for the virtual joystick widget."""

from __future__ import annotations

import math

from .config import AXIS_DEADBAND
from .joystick_math import JoystickGeometry


def draw_virtual_joystick(
    canvas,
    *,
    geometry: JoystickGeometry,
    title: str,
    x_label: str,
    y_label: str,
    x: float,
    y: float,
    magnitude: float,
) -> None:
    canvas.delete("all")
    _draw_background(canvas, geometry, title, x_label, y_label, x, y)
    base_r, gate_r = _draw_base(canvas, geometry)
    knob_x = geometry.cx + x * geometry.radius
    knob_y = geometry.cy - y * geometry.radius
    _draw_knob(canvas, geometry, knob_x, knob_y, magnitude)
    _draw_axis_labels(canvas, geometry, base_r, x_label, y_label)


def _draw_background(canvas, geometry: JoystickGeometry, title: str, x_label: str, y_label: str, x: float, y: float) -> None:
    canvas.create_rectangle(0, 0, geometry.width, geometry.height, fill="#111827", outline="")
    canvas.create_text(
        12,
        12,
        text=title,
        fill="#e5e7eb",
        anchor="nw",
        font=("TkDefaultFont", 10, "bold"),
    )
    canvas.create_text(
        geometry.width - 12,
        12,
        text=f"{y_label} {y:+.2f}   {x_label} {x:+.2f}",
        fill="#94a3b8",
        anchor="ne",
        font=("TkDefaultFont", 8),
    )


def _draw_base(canvas, geometry: JoystickGeometry) -> tuple[float, float]:
    cx, cy, radius = geometry.cx, geometry.cy, geometry.radius
    base_r = radius * 1.42
    gate_r = radius * 1.02
    canvas.create_oval(cx - base_r + 3, cy - base_r + 6, cx + base_r + 3, cy + base_r + 6, fill="#020617", outline="")
    canvas.create_oval(cx - base_r, cy - base_r, cx + base_r, cy + base_r, fill="#1f2937", outline="#475569", width=2)
    for idx, color in enumerate(("#182235", "#141d2e", "#101827", "#0b1322")):
        r = base_r * (0.86 - idx * 0.13)
        canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color, outline="")
    canvas.create_oval(cx - gate_r, cy - gate_r, cx + gate_r, cy + gate_r, outline="#64748b", width=2)
    canvas.create_oval(
        cx - gate_r * 0.55,
        cy - gate_r * 0.55,
        cx + gate_r * 0.55,
        cy + gate_r * 0.55,
        outline="#263549",
        width=1,
    )
    _draw_cardinal_notches(canvas, geometry, base_r)
    return base_r, gate_r


def _draw_cardinal_notches(canvas, geometry: JoystickGeometry, base_r: float) -> None:
    notch_r = base_r * 0.93
    notch_len = 8
    for angle_deg in (0, 90, 180, 270):
        angle = math.radians(angle_deg)
        x0 = geometry.cx + math.cos(angle) * (notch_r - notch_len)
        y0 = geometry.cy + math.sin(angle) * (notch_r - notch_len)
        x1 = geometry.cx + math.cos(angle) * notch_r
        y1 = geometry.cy + math.sin(angle) * notch_r
        canvas.create_line(x0, y0, x1, y1, fill="#94a3b8", width=2)


def _draw_knob(canvas, geometry: JoystickGeometry, knob_x: float, knob_y: float, magnitude: float) -> None:
    shaft_r = geometry.radius * 0.42
    canvas.create_oval(
        knob_x - shaft_r + 2,
        knob_y - shaft_r + 4,
        knob_x + shaft_r + 2,
        knob_y + shaft_r + 4,
        fill="#020617",
        outline="",
    )
    canvas.create_line(geometry.cx, geometry.cy, knob_x, knob_y, fill="#0f7490", width=4 if magnitude else 2)
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


def _draw_axis_labels(canvas, geometry: JoystickGeometry, base_r: float, x_label: str, y_label: str) -> None:
    label_r = base_r * 1.10
    cx, cy = geometry.cx, geometry.cy
    canvas.create_text(cx, cy - label_r, text=y_label, fill="#cbd5e1", font=("TkDefaultFont", 9, "bold"))
    canvas.create_text(cx, cy + label_r, text=f"-{y_label}", fill="#64748b", font=("TkDefaultFont", 9))
    canvas.create_text(cx + label_r, cy, text=x_label, fill="#cbd5e1", anchor="w", font=("TkDefaultFont", 9, "bold"))
    canvas.create_text(cx - label_r, cy, text=f"-{x_label}", fill="#64748b", anchor="e", font=("TkDefaultFont", 9))


__all__ = ["draw_virtual_joystick"]
