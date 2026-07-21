"""Mode shading helpers for ALT_HOLD contract plots."""

from __future__ import annotations


def shade_modes(ax, modes: list[dict], end_t: float) -> None:
    if not modes:
        return
    colors = {"ALT_HOLD": "#ffe6e6", "MANUAL": "#e8f2ff"}
    for i, mode in enumerate(modes):
        start = float(mode["t"])
        end = float(modes[i + 1]["t"]) if i + 1 < len(modes) else end_t
        ax.axvspan(start, end, color=colors.get(mode["name"], "#f3f3f3"), alpha=0.35, linewidth=0)
        ax.text(start + 0.4, 0.96, str(mode["name"]), transform=ax.get_xaxis_transform(), fontsize=8, va="top")


def prepare_axes(axes, modes: list[dict], end_t: float) -> None:
    for ax in axes:
        shade_modes(ax, modes, end_t)
        ax.grid(True, alpha=0.25)


__all__ = ["prepare_axes", "shade_modes"]
