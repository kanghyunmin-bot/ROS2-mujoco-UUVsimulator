"""Text payload construction for the GUI telemetry refresh loop."""

from __future__ import annotations

from .control_update_pilot_texts import build_pilot_control_texts
from .control_update_telemetry_texts import build_telemetry_texts
from .control_update_text_models import ControlUpdateTexts


def build_control_update_texts(
    *,
    snap,
    commands,
    backend_label: str,
    rc_mapping_summary: str,
    mode_display: str,
    control_mode: str,
    control_details_visible: bool,
    rc_override_enabled: bool,
    vehicle_info_supported: bool,
) -> ControlUpdateTexts:
    telemetry = build_telemetry_texts(
        snap=snap,
        backend_label=backend_label,
        rc_mapping_summary=rc_mapping_summary,
        mode_display=mode_display,
        vehicle_info_supported=vehicle_info_supported,
    )
    pilot = build_pilot_control_texts(
        snap=snap,
        commands=commands,
        rc_mapping_summary=rc_mapping_summary,
        control_mode=control_mode,
        control_details_visible=control_details_visible,
        rc_override_enabled=rc_override_enabled,
    )
    return ControlUpdateTexts(**telemetry.__dict__, **pilot.__dict__)


__all__ = ["ControlUpdateTexts", "build_control_update_texts"]
