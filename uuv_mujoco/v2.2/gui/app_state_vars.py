"""Tk variable initialization facade for the UUV control GUI."""

from __future__ import annotations

from .app_state_core_vars import (
    initialize_command_axis_vars,
    initialize_vehicle_status_vars,
    initialize_visibility_vars,
)
from .app_state_feature_vars import (
    initialize_physics_vars,
    initialize_ping360_vars,
    initialize_replay_vars,
    initialize_runtime_panel_vars,
)


def initialize_gui_vars(owner) -> None:
    initialize_visibility_vars(owner)
    initialize_command_axis_vars(owner)
    initialize_vehicle_status_vars(owner)
    initialize_replay_vars(owner)
    initialize_physics_vars(owner)
    initialize_runtime_panel_vars(owner)
    initialize_ping360_vars(owner)


__all__ = ["initialize_gui_vars"]
