"""Telemetry panel layout helpers for GUI visibility toggles."""

from __future__ import annotations

from .config import TELEMETRY_HIDDEN_MINSIZE, TELEMETRY_HIDDEN_WIDTH, WINDOW_MINSIZE


def apply_telemetry_panel_visibility(owner, *, show: bool) -> None:
    if owner.main_container is None or owner.telemetry_panel is None or owner.control_panel is None:
        return
    if show:
        show_telemetry_panel(owner)
    else:
        hide_telemetry_panel(owner)


def show_telemetry_panel(owner) -> None:
    owner.root.minsize(*WINDOW_MINSIZE)
    owner.telemetry_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
    owner.control_panel.grid_configure(row=0, column=1, columnspan=1, sticky="nsew")
    owner.main_container.columnconfigure(0, weight=3)
    owner.main_container.columnconfigure(1, weight=2)
    if owner.root.winfo_width() < WINDOW_MINSIZE[0]:
        owner.root.geometry(f"{WINDOW_MINSIZE[0]}x{max(owner.root.winfo_height(), WINDOW_MINSIZE[1])}")
    owner.telemetry_toggle_button.config(text="Hide telemetry")


def hide_telemetry_panel(owner) -> None:
    owner.telemetry_panel.grid_remove()
    owner.control_panel.grid_configure(row=0, column=0, columnspan=2, sticky="nsew")
    owner.main_container.columnconfigure(0, weight=1)
    owner.main_container.columnconfigure(1, weight=0)
    owner.root.minsize(*TELEMETRY_HIDDEN_MINSIZE)
    owner.root.geometry(
        f"{TELEMETRY_HIDDEN_WIDTH}x{max(owner.root.winfo_height(), TELEMETRY_HIDDEN_MINSIZE[1])}"
    )
    owner.telemetry_toggle_button.config(text="Show telemetry")


__all__ = ["apply_telemetry_panel_visibility", "hide_telemetry_panel", "show_telemetry_panel"]
