"""Runtime setup and callbacks for the virtual joystick widget."""

from __future__ import annotations

import tkinter as tk
from typing import Callable

from .joystick_math import axes_from_canvas_event, display_axes, joystick_geometry, stick_axes
from .joystick_render import draw_virtual_joystick


def initialize_virtual_joystick(
    owner,
    *,
    title: str,
    x_var: tk.DoubleVar,
    y_var: tk.DoubleVar,
    x_label: str,
    y_label: str,
    on_change: Callable[[], None] | None,
) -> None:
    owner.x_var = x_var
    owner.y_var = y_var
    owner.x_label = x_label
    owner.y_label = y_label
    owner.on_change = on_change
    owner.title = title
    owner.columnconfigure(0, weight=1)
    owner.rowconfigure(0, weight=1)
    owner.canvas = create_joystick_canvas(owner)
    bind_virtual_joystick(owner)
    redraw_virtual_joystick(owner)


def create_joystick_canvas(owner):
    canvas = tk.Canvas(
        owner,
        width=joystick_geometry().width,
        height=joystick_geometry().height,
        bg="#111827",
        highlightthickness=0,
        bd=0,
    )
    canvas.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
    return canvas


def bind_virtual_joystick(owner) -> None:
    owner.canvas.bind("<Configure>", lambda _event: redraw_virtual_joystick(owner))
    owner.canvas.bind("<Button-1>", lambda event: update_joystick_from_drag(owner, event))
    owner.canvas.bind("<B1-Motion>", lambda event: update_joystick_from_drag(owner, event))
    owner.canvas.bind("<ButtonRelease-1>", lambda _event: center_virtual_joystick(owner))
    owner.canvas.bind("<Double-Button-1>", lambda _event: center_virtual_joystick(owner))
    owner.x_var.trace_add("write", lambda *_args: redraw_virtual_joystick(owner))
    owner.y_var.trace_add("write", lambda *_args: redraw_virtual_joystick(owner))


def joystick_canvas_geometry(owner):
    return joystick_geometry(owner.canvas.winfo_width(), owner.canvas.winfo_height())


def set_virtual_joystick_axes(owner, x: float, y: float) -> None:
    x, y = stick_axes(x, y)
    owner.x_var.set(x)
    owner.y_var.set(y)
    if owner.on_change is not None:
        owner.on_change()


def update_joystick_from_drag(owner, event) -> None:
    set_virtual_joystick_axes(
        owner,
        *axes_from_canvas_event(event.x, event.y, joystick_canvas_geometry(owner)),
    )


def center_virtual_joystick(owner) -> None:
    set_virtual_joystick_axes(owner, 0.0, 0.0)


def redraw_virtual_joystick(owner) -> None:
    x, y, magnitude = display_axes(owner.x_var.get(), owner.y_var.get())
    draw_virtual_joystick(
        owner.canvas,
        geometry=joystick_canvas_geometry(owner),
        title=owner.title,
        x_label=owner.x_label,
        y_label=owner.y_label,
        x=x,
        y=y,
        magnitude=magnitude,
    )


__all__ = [
    "center_virtual_joystick",
    "initialize_virtual_joystick",
    "redraw_virtual_joystick",
    "set_virtual_joystick_axes",
    "update_joystick_from_drag",
]
