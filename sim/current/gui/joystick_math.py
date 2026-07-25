"""Geometry and axis math for the virtual joystick widget."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import AXIS_DEADBAND, AXIS_MAX, AXIS_MIN, JOYSTICK_CANVAS_SIZE


@dataclass(frozen=True)
class JoystickGeometry:
    width: int
    height: int
    cx: float
    cy: float
    radius: float


def joystick_geometry(
    canvas_width: int = JOYSTICK_CANVAS_SIZE,
    canvas_height: int = JOYSTICK_CANVAS_SIZE + 18,
) -> JoystickGeometry:
    width = max(canvas_width, JOYSTICK_CANVAS_SIZE)
    height = max(canvas_height, JOYSTICK_CANVAS_SIZE + 18)
    cx = width / 2.0
    cy = height / 2.0 + 10.0
    radius = min(width, height - 18) * 0.29
    return JoystickGeometry(width=width, height=height, cx=cx, cy=cy, radius=radius)


def normalize_unit_axes(x: float, y: float) -> tuple[float, float, float]:
    magnitude = math.hypot(x, y)
    if magnitude > 1.0:
        x /= magnitude
        y /= magnitude
        magnitude = 1.0
    return x, y, magnitude


def clamp_axis(value: float) -> float:
    return max(AXIS_MIN, min(AXIS_MAX, float(value)))


def stick_axes(x: float, y: float) -> tuple[float, float]:
    if abs(x) < AXIS_DEADBAND:
        x = 0.0
    if abs(y) < AXIS_DEADBAND:
        y = 0.0
    return clamp_axis(x), clamp_axis(y)


def display_axes(x: float, y: float) -> tuple[float, float, float]:
    return normalize_unit_axes(clamp_axis(x), clamp_axis(y))


def axes_from_canvas_event(event_x: float, event_y: float, geometry: JoystickGeometry) -> tuple[float, float]:
    x = (event_x - geometry.cx) / geometry.radius
    y = (geometry.cy - event_y) / geometry.radius
    x, y, _ = normalize_unit_axes(x, y)
    return x, y


__all__ = [
    "JoystickGeometry",
    "joystick_geometry",
    "normalize_unit_axes",
    "clamp_axis",
    "stick_axes",
    "display_axes",
    "axes_from_canvas_event",
]
