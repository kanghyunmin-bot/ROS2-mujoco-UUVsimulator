"""Ping360 control window construction."""

from __future__ import annotations

from .ping360_window_lifecycle import (
    _close_ping360_window,
    _show_ping360_window,
    _toggle_ping360_window,
)
from .ping360_window_panels import (
    _build_ping360_footer,
    _build_ping360_params_panel,
    _build_ping360_power_panel,
    _build_ping360_status_panel,
    _build_ping360_view_panel,
)


class Ping360WindowMixin:
    _toggle_ping360_window = _toggle_ping360_window
    _show_ping360_window = _show_ping360_window
    _close_ping360_window = _close_ping360_window
    _build_ping360_status_panel = _build_ping360_status_panel
    _build_ping360_power_panel = _build_ping360_power_panel
    _build_ping360_view_panel = _build_ping360_view_panel
    _build_ping360_params_panel = _build_ping360_params_panel
    _build_ping360_footer = _build_ping360_footer


__all__ = ["Ping360WindowMixin"]
