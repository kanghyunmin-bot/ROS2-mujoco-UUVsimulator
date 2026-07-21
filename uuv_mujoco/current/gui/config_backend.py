"""Backend and pilot-control mode configuration for the MuJoCo UUV GUI."""

from __future__ import annotations

import os


BACKEND_AUTO = "auto"
BACKEND_NONE = "none"
BACKEND_MAVROS = "mavros"
BACKEND_SIM_BRIDGE = "sim_bridge"
DEFAULT_AUTO_BACKEND = BACKEND_NONE

PILOT_CONTROL_MANUAL = "manual_control"
PILOT_CONTROL_RC_OVERRIDE = "rc_override"


def pilot_control_mode() -> str:
    default_mode = os.environ.get(
        "UUV_GUI_DEFAULT_PILOT_CONTROL_MODE",
        PILOT_CONTROL_RC_OVERRIDE,
    ).strip().lower()
    mode = os.environ.get("UUV_GUI_PILOT_CONTROL_MODE", default_mode).strip().lower()
    if mode in {PILOT_CONTROL_MANUAL, PILOT_CONTROL_RC_OVERRIDE}:
        return mode
    return PILOT_CONTROL_RC_OVERRIDE


GUI_PILOT_CONTROL_MODE = pilot_control_mode()


__all__ = [
    "BACKEND_AUTO",
    "BACKEND_MAVROS",
    "BACKEND_NONE",
    "BACKEND_SIM_BRIDGE",
    "DEFAULT_AUTO_BACKEND",
    "GUI_PILOT_CONTROL_MODE",
    "PILOT_CONTROL_MANUAL",
    "PILOT_CONTROL_RC_OVERRIDE",
    "pilot_control_mode",
]
