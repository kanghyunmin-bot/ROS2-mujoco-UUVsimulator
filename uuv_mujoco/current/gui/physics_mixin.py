"""Compatibility mixin for GUI physics parameter editing."""

from __future__ import annotations

from .physics_param_io import (
    _apply_physics_params,
    _current_physics_profile,
    _format_physics_value,
    _get_nested_value,
    _load_physics_params_into_fields,
    _parse_physics_number,
    _parse_physics_value,
    _physics_current_mode_status,
    _physics_param_inactive_in_current,
    _set_nested_value,
)
from .physics_restart import _restart_sim_stack_after_physics_apply
from .physics_window import _close_physics_window, _set_physics_status, _show_physics_window


class PhysicsMixin:
    _show_physics_window = _show_physics_window
    _close_physics_window = _close_physics_window
    _set_physics_status = _set_physics_status
    _load_physics_params_into_fields = _load_physics_params_into_fields
    _parse_physics_value = _parse_physics_value
    _apply_physics_params = _apply_physics_params
    _restart_sim_stack_after_physics_apply = _restart_sim_stack_after_physics_apply

    _physics_param_inactive_in_current = staticmethod(_physics_param_inactive_in_current)
    _physics_current_mode_status = staticmethod(_physics_current_mode_status)
    _format_physics_value = staticmethod(_format_physics_value)
    _get_nested_value = staticmethod(_get_nested_value)
    _set_nested_value = staticmethod(_set_nested_value)
    _current_physics_profile = staticmethod(_current_physics_profile)
    _parse_physics_number = staticmethod(_parse_physics_number)
