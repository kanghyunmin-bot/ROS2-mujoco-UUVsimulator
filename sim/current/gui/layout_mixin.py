"""Tk layout construction for the MuJoCo UUV GUI."""

from __future__ import annotations

from .layout_control_core import build_control_panel
from .layout_control_pilot import build_pilot_section
from .layout_control_replay import build_rc_replay_section
from .layout_control_tools import build_control_tools_panel
from .layout_control_tuning import build_tuning_sections
from .layout_shell import build_layout_shell
from .layout_telemetry import build_telemetry_panel


class LayoutMixin:
    def _build_layout(self) -> None:
        left, right = build_layout_shell(self)
        build_telemetry_panel(self, left)
        control_box = build_control_panel(self, right)
        control_tools = build_control_tools_panel(self, right)
        build_rc_replay_section(self, control_tools, row=0)
        build_tuning_sections(self, control_tools, start_row=1)
        build_pilot_section(self, control_box)
