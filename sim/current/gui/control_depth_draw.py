"""Depth widget canvas drawing."""

from __future__ import annotations

import math

from .config import MAX_DEPTH_DISPLAY_M


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def depth_display_values(depth_m: float, source: str) -> tuple[bool, float, str, str]:
    value_valid = math.isfinite(depth_m)
    display_depth = depth_m if value_valid else 0.0
    clamped_depth = _clamp(display_depth, 0.0, MAX_DEPTH_DISPLAY_M)
    value_text = f"{depth_m:.2f} m" if value_valid else "n/a"
    source_text = source if len(source) <= 28 else f"{source[:25]}..."
    return value_valid, clamped_depth, value_text, source_text


def draw_depth_canvas(canvas, depth_m: float, source: str) -> None:
    canvas.delete("all")
    width = max(canvas.winfo_width(), 140)
    height = max(canvas.winfo_height(), 82)
    pad = 12
    value_valid, clamped_depth, value_text, source_text = depth_display_values(depth_m, source)
    ratio = clamped_depth / max(MAX_DEPTH_DISPLAY_M, 1e-6)

    canvas.create_rectangle(0, 0, width, height, fill="#0f172a", outline="")
    draw_depth_labels(canvas, width, pad, value_valid, value_text, source_text)
    draw_depth_bar(canvas, width, height, pad, ratio, value_valid)


def draw_depth_labels(
    canvas,
    width: int,
    pad: int,
    value_valid: bool,
    value_text: str,
    source_text: str,
) -> None:
    canvas.create_text(pad, 11, anchor="nw", fill="#94a3b8", font=("TkDefaultFont", 9, "bold"), text="DEPTH")
    canvas.create_text(
        pad,
        31,
        anchor="w",
        fill="#e0f2fe" if value_valid else "#64748b",
        font=("TkDefaultFont", 20, "bold"),
        text=value_text,
    )
    canvas.create_text(width - pad, 15, anchor="ne", fill="#64748b", font=("TkDefaultFont", 8), text=source_text)


def draw_depth_bar(
    canvas,
    width: int,
    height: int,
    pad: int,
    ratio: float,
    value_valid: bool,
) -> None:
    bar_x0 = pad
    bar_x1 = width - pad
    bar_y0 = height - 26
    bar_y1 = height - 15
    canvas.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1, fill="#1e293b", outline="#334155")
    if value_valid:
        canvas.create_rectangle(
            bar_x0 + 1,
            bar_y0 + 1,
            bar_x0 + 1 + (bar_x1 - bar_x0 - 2) * ratio,
            bar_y1 - 1,
            fill="#38bdf8",
            outline="",
        )
    canvas.create_text(bar_x0, height - 6, anchor="sw", fill="#94a3b8", font=("TkDefaultFont", 8), text="0")
    canvas.create_text(
        bar_x1,
        height - 6,
        anchor="se",
        fill="#94a3b8",
        font=("TkDefaultFont", 8),
        text=f"{MAX_DEPTH_DISPLAY_M:g} m",
    )


__all__ = [
    "depth_display_values",
    "draw_depth_bar",
    "draw_depth_canvas",
    "draw_depth_labels",
]
