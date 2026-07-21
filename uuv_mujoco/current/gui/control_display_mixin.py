"""Compatibility facade for GUI manual control and telemetry display."""

from __future__ import annotations

from .control_draw_mixin import ControlDrawMixin
from .control_feedback_mixin import ControlFeedbackMixin
from .control_pilot_mixin import ControlPilotMixin
from .control_toggle_mixin import ControlToggleMixin
from .control_update_mixin import ControlUpdateMixin


class ControlDisplayMixin(
    ControlPilotMixin,
    ControlToggleMixin,
    ControlDrawMixin,
    ControlFeedbackMixin,
    ControlUpdateMixin,
):
    """Preserve the historical GUI mixin API while keeping roles separated."""
