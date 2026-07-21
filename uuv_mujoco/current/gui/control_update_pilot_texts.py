"""Pilot-control text construction for the GUI refresh loop."""

from __future__ import annotations

from .config import GUI_PILOT_CONTROL_MODE
from .config_backend import PILOT_CONTROL_RC_OVERRIDE
from .control_update_format import axis_to_pwm, pilot_heave_axis_summary
from .control_update_text_models import PilotControlTexts


def pilot_input_source_text(*, snap, commands) -> str:
    if GUI_PILOT_CONTROL_MODE == PILOT_CONTROL_RC_OVERRIDE:
        return f"rcin3={snap.rc_in[2] if len(snap.rc_in) > 2 else 0}"
    return (
        "manual="
        f"x={commands.rc_forward:+.2f},"
        f"y={commands.rc_lateral:+.2f},"
        f"z={commands.rc_heave:+.2f},"
        f"r={commands.rc_yaw:+.2f}"
    )


def build_pilot_control_texts(
    *,
    snap,
    commands,
    rc_mapping_summary: str,
    control_mode: str,
    control_details_visible: bool,
    rc_override_enabled: bool,
) -> PilotControlTexts:
    heave_mode, heave_target_vz = pilot_heave_axis_summary(commands.rc_heave)
    return PilotControlTexts(
        control_summary=f"control: {control_mode}  details={'shown' if control_details_visible else 'hidden'}",
        control=(
            f"rc setpoint: fwd={commands.rc_forward:+.2f}  lat={commands.rc_lateral:+.2f}  "
            f"heave={commands.rc_heave:+.2f}  yaw={commands.rc_yaw:+.2f}"
        ),
        rc_override=(
            "pilot input: "
            f"{'on' if rc_override_enabled else 'off'}  "
            f"{GUI_PILOT_CONTROL_MODE}  "
            f"{rc_mapping_summary}  "
            f"heave={heave_mode} "
            f"target_vz={heave_target_vz:+.1f}cm/s  "
            f"yaw={axis_to_pwm(commands.rc_yaw)}  "
            f"forward={axis_to_pwm(commands.rc_forward)}  lateral={axis_to_pwm(commands.rc_lateral)}  "
            f"{pilot_input_source_text(snap=snap, commands=commands)}  "
            f"rcout5-8={tuple(snap.rc_out[4:8])}  "
            f"feedback={snap.rc_feedback_source}"
        ),
    )


__all__ = ["build_pilot_control_texts", "pilot_input_source_text"]
