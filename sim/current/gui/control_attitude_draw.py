"""Attitude widget canvas drawing."""

from __future__ import annotations

import math


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _rotator(cx: float, cy: float, cos_r: float, sin_r: float):
    def rot(x: float, y: float) -> tuple[float, float]:
        return (cx + x * cos_r - y * sin_r, cy + x * sin_r + y * cos_r)

    return rot


def _flatten_points(points: list[tuple[float, float]]) -> list[float]:
    return [coord for point in points for coord in point]


def draw_attitude_canvas(canvas, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 100)
    height = max(canvas.winfo_height(), 100)
    cx = width / 2.0
    cy = height / 2.0

    pitch_offset = _clamp(pitch_deg, -45.0, 45.0) * 2.2
    roll_rad = math.radians(roll_deg)
    extent = max(width, height) * 1.8
    half = extent / 2.0
    rot = _rotator(cx, cy, math.cos(roll_rad), math.sin(roll_rad))

    sky = [
        rot(-half, -half - pitch_offset),
        rot(half, -half - pitch_offset),
        rot(half, -pitch_offset),
        rot(-half, -pitch_offset),
    ]
    ground = [
        rot(-half, -pitch_offset),
        rot(half, -pitch_offset),
        rot(half, half - pitch_offset),
        rot(-half, half - pitch_offset),
    ]
    canvas.create_polygon(*_flatten_points(sky), fill="#1d4ed8", outline="")
    canvas.create_polygon(*_flatten_points(ground), fill="#854d0e", outline="")

    left = rot(-half, -pitch_offset)
    right = rot(half, -pitch_offset)
    canvas.create_line(left[0], left[1], right[0], right[1], fill="white", width=3)
    draw_pitch_ladder(canvas, rot, pitch_offset)
    draw_aircraft_symbol(canvas, cx, cy)
    draw_attitude_text(canvas, roll_deg, pitch_deg, yaw_deg)


def draw_pitch_ladder(canvas, rot, pitch_offset: float) -> None:
    for step in range(-30, 35, 10):
        if step == 0:
            continue
        y_line = -pitch_offset - step * 2.2
        span = 60 if step % 20 == 0 else 30
        p1 = rot(-span, y_line)
        p2 = rot(span, y_line)
        canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill="#e2e8f0", width=2)


def draw_aircraft_symbol(canvas, cx: float, cy: float) -> None:
    canvas.create_line(cx - 70, cy, cx - 15, cy, fill="#f8fafc", width=4)
    canvas.create_line(cx + 15, cy, cx + 70, cy, fill="#f8fafc", width=4)
    canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, outline="#f8fafc", width=2)
    canvas.create_line(cx, cy - 18, cx, cy + 18, fill="#f8fafc", width=2)


def draw_attitude_text(canvas, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
    canvas.create_text(
        12,
        12,
        anchor="nw",
        fill="#f8fafc",
        font=("TkDefaultFont", 12, "bold"),
        text=f"ROLL {roll_deg:+05.1f}  PITCH {pitch_deg:+05.1f}  YAW {yaw_deg:+06.1f}",
    )


__all__ = [
    "draw_aircraft_symbol",
    "draw_attitude_canvas",
    "draw_attitude_text",
    "draw_pitch_ladder",
]
