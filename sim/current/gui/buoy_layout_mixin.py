"""Compatibility mixin for course-buoy XY layout editing."""

from __future__ import annotations

from .buoy_layout_window import _show_buoy_layout_window


class BuoyLayoutMixin:
    _show_buoy_layout_window = _show_buoy_layout_window


__all__ = ["BuoyLayoutMixin"]
