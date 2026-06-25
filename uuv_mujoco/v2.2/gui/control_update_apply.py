"""Apply telemetry-refresh text payloads to GUI variables."""

from __future__ import annotations

from .control_update_text_models import ControlUpdateTexts


def apply_control_update_texts(owner, texts: ControlUpdateTexts) -> None:
    owner.status_var.set(texts.status)
    owner.mode_var.set(texts.mode)
    owner.vehicle_summary_var.set(texts.vehicle_summary)
    owner.battery_var.set(texts.battery)
    owner.pose_var.set(texts.pose)
    owner.vel_var.set(texts.velocity)
    owner.imu_var.set(texts.imu)
    owner.motion_summary_var.set(texts.motion_summary)
    owner.autopilot_var.set(texts.autopilot)
    owner.depth_target_var.set(texts.depth_target)
    owner.depth_source_var.set(texts.depth_source)
    owner.age_var.set(texts.age)
    owner.ping360_summary_var.set(texts.ping360_summary)
    owner.control_summary_var.set(texts.control_summary)
    owner.control_var.set(texts.control)
    owner.rc_override_var.set(texts.rc_override)


def apply_command_ready(owner, text: str, style: str) -> None:
    owner.command_ready_var.set(text)
    if owner.command_ready_label is not None:
        owner.command_ready_label.configure(style=style)


def apply_ping360_enabled(owner, enabled: bool | None) -> None:
    if enabled is not None:
        owner.ping360_enabled_var.set(bool(enabled))


__all__ = ["apply_command_ready", "apply_control_update_texts", "apply_ping360_enabled"]
