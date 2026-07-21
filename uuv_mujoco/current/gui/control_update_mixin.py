"""Main GUI telemetry refresh loop."""

from __future__ import annotations

from .control_update_apply import apply_command_ready, apply_control_update_texts, apply_ping360_enabled
from .control_update_command_ready import command_ready_for_gui_stack
from .control_update_mode import active_control_mode
from .control_update_texts import build_control_update_texts
from .control_update_vehicle import request_vehicle_info_if_due
from .runtime import time


class ControlUpdateMixin:
    def _update_ui(self) -> None:
        if self._closed or not self.root.winfo_exists():
            return
        snap = self.node.snapshot()
        self.node.probe_backend()
        now = time.monotonic()
        request_vehicle_info_if_due(self, now=now)

        backend_label = self.node.backend_label()
        rc_mapping_summary = self.node.rc_mapping_summary()
        mode_display = snap.vehicle_mode or snap.mode
        self._refresh_sim_stack_status()
        command_ready_text, command_ready_style = self.node.control_readiness(snap)
        command_ready_text, command_ready_style = command_ready_for_gui_stack(
            self,
            command_ready_text,
            command_ready_style,
        )
        apply_command_ready(self, command_ready_text, command_ready_style)

        commands = self._read_control_commands()
        control_mode = active_control_mode(self)
        texts = build_control_update_texts(
            snap=snap,
            commands=commands,
            backend_label=backend_label,
            rc_mapping_summary=rc_mapping_summary,
            mode_display=mode_display,
            control_mode=control_mode,
            control_details_visible=self.control_details_visible.get(),
            rc_override_enabled=self.rc_override_enabled.get(),
            vehicle_info_supported=self.node.vehicle_info_supported(),
        )
        apply_control_update_texts(self, texts)
        apply_ping360_enabled(self, snap.ping360_enabled)
        self._refresh_ros2_buttons()

        self._draw_attitude(snap.roll_deg, snap.pitch_deg, snap.yaw_deg)
        self._draw_depth(snap.depth_m, snap.depth_source)
        self._update_events(snap.events)

        feedback_channels = snap.rc_in if self._has_rc_feedback(snap.rc_in) else snap.rc_out
        self._update_rc_feedback_bars(feedback_channels)

        self._schedule_update()
