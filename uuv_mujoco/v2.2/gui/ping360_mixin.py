"""Compatibility facade for Ping360 GUI controls."""

from __future__ import annotations

from .ping360_config_mixin import Ping360ConfigMixin
from .ping360_view_mixin import Ping360ViewMixin
from .ping360_window_mixin import Ping360WindowMixin


class Ping360ControlMixin(Ping360WindowMixin, Ping360ViewMixin, Ping360ConfigMixin):
    pass


__all__ = ["Ping360ControlMixin"]
