"""Compatibility facade for GUI RC contract helpers."""

from __future__ import annotations

from .gui_rc_althold import (
    althold_level_climb_rate_from_rc3_pwm,
    effective_js_gain,
    manual_heave_axis_to_rc3_pwm,
    pilot_heave_axis_summary,
)
from .gui_rc_axes import gui_rc_to_override_axes
from .gui_rc_messages import (
    make_rc_override_message,
    make_rc_release_message,
    padded_rc_channels,
    sanitize_primary_rc_override_channels,
)
from .gui_rc_pwm import axis_to_pwm, heave_axis_to_rc3_pwm, valid_rc_pwm


__all__ = [
    "axis_to_pwm",
    "heave_axis_to_rc3_pwm",
    "valid_rc_pwm",
    "sanitize_primary_rc_override_channels",
    "effective_js_gain",
    "manual_heave_axis_to_rc3_pwm",
    "althold_level_climb_rate_from_rc3_pwm",
    "pilot_heave_axis_summary",
    "gui_rc_to_override_axes",
    "make_rc_override_message",
    "make_rc_release_message",
    "padded_rc_channels",
]
