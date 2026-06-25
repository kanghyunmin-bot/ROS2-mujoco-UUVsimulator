"""RC, pilot, and command-axis GUI configuration."""

from __future__ import annotations

import os
from dataclasses import dataclass

from sim.contracts import (
    PRIMARY_RC_CHANNEL_COUNT as CONTRACT_PRIMARY_RC_CHANNEL_COUNT,
    PWM_CENTER,
    RC_OVERRIDE_CHANNEL_COUNT,
    RC_VALID_MAX_PWM as CONTRACT_RC_VALID_MAX_PWM,
    RC_VALID_MIN_PWM as CONTRACT_RC_VALID_MIN_PWM,
)

from .config_backend import BACKEND_MAVROS, BACKEND_NONE, BACKEND_SIM_BRIDGE
from .config_env import env_float_default


AXIS_MIN = -1.0
AXIS_MAX = 1.0
AXIS_DEADBAND = 0.03
RC_NEUTRAL_PWM = PWM_CENTER
RC_PWM_SPAN = float(os.environ.get("UUV_GUI_RC_PWM_SPAN", "300.0"))
RC_VALID_MIN_PWM = CONTRACT_RC_VALID_MIN_PWM
RC_VALID_MAX_PWM = CONTRACT_RC_VALID_MAX_PWM
RC_MESSAGE_CHANNEL_COUNT = RC_OVERRIDE_CHANNEL_COUNT
RC_FEEDBACK_CHANNEL_COUNT = 16
RC_VISIBLE_CHANNEL_COUNT = 8
PRIMARY_RC_CHANNEL_COUNT = CONTRACT_PRIMARY_RC_CHANNEL_COUNT
RC_HEAVE_CHANNEL_INDEX = 2

REAL_JS_GAIN_DEFAULT = env_float_default("SITL_JS_GAIN_DEFAULT", 0.5)
REAL_JS_GAIN_MIN = env_float_default("SITL_JS_GAIN_MIN", 0.25)
REAL_JS_GAIN_MAX = env_float_default("SITL_JS_GAIN_MAX", 2.0)
REAL_JS_GAIN_STEPS = int(env_float_default("SITL_JS_GAIN_STEPS", 4))
REAL_JS_THR_GAIN = env_float_default("SITL_JS_THR_GAIN", 1.0)
REAL_RC3_MIN = int(env_float_default("SITL_RC3_MIN", 1100))
REAL_RC3_MAX = int(env_float_default("SITL_RC3_MAX", 1900))
REAL_RC3_TRIM = int(env_float_default("SITL_RC3_TRIM", 1100))
REAL_RC3_DZ = int(env_float_default("SITL_RC3_DZ", 30))
REAL_PILOT_SPEED_UP = env_float_default("SITL_PILOT_SPEED_UP", 100)
REAL_PILOT_SPEED_DN = env_float_default("SITL_PILOT_SPEED_DN", 0)
ALT_HOLD_RC_HEAVE_INVERT = (
    os.environ.get("UUV_ALT_HOLD_RC_HEAVE_INVERT", "0").strip().lower()
    in {"1", "true", "yes", "on", "enable", "enabled"}
)


@dataclass(frozen=True)
class RcLayout:
    label: str
    axis_channels: dict[str, int]
    summary: str


RC_LAYOUTS = {
    BACKEND_NONE: RcLayout(
        label="no ROS bridge detected",
        axis_channels={
            "heave": 2,    # ch3
            "yaw": 3,      # ch4
            "forward": 4,  # ch5
            "lateral": 5,  # ch6
        },
        summary="control unavailable until --ros2 or external MAVROS is running",
    ),
    BACKEND_MAVROS: RcLayout(
        label="legacy ArduSub/MAVROS",
        axis_channels={
            "heave": 2,    # ch3
            "yaw": 3,      # ch4
            "forward": 4,  # ch5
            "lateral": 5,  # ch6
        },
        summary="ch3=heave, ch4=yaw, ch5=forward, ch6=lateral",
    ),
    BACKEND_SIM_BRIDGE: RcLayout(
        label="MuJoCo sim bridge",
        axis_channels={
            "heave": 2,    # ch3
            "yaw": 3,      # ch4
            "forward": 4,  # ch5
            "lateral": 5,  # ch6
        },
        summary="ch3=heave, ch4=yaw, ch5=forward, ch6=lateral",
    ),
}

CMDVEL_AXIS_SCALE = {
    "forward": 0.40,
    "lateral": 0.35,
    "heave": 0.25,
    "yaw": 0.45,
}


__all__ = [
    "ALT_HOLD_RC_HEAVE_INVERT",
    "AXIS_DEADBAND",
    "AXIS_MAX",
    "AXIS_MIN",
    "CMDVEL_AXIS_SCALE",
    "PRIMARY_RC_CHANNEL_COUNT",
    "RC_FEEDBACK_CHANNEL_COUNT",
    "RC_HEAVE_CHANNEL_INDEX",
    "RC_LAYOUTS",
    "RC_MESSAGE_CHANNEL_COUNT",
    "RC_NEUTRAL_PWM",
    "RC_PWM_SPAN",
    "RC_VALID_MAX_PWM",
    "RC_VALID_MIN_PWM",
    "RC_VISIBLE_CHANNEL_COUNT",
    "REAL_JS_GAIN_DEFAULT",
    "REAL_JS_GAIN_MAX",
    "REAL_JS_GAIN_MIN",
    "REAL_JS_GAIN_STEPS",
    "REAL_JS_THR_GAIN",
    "REAL_PILOT_SPEED_DN",
    "REAL_PILOT_SPEED_UP",
    "REAL_RC3_DZ",
    "REAL_RC3_MAX",
    "REAL_RC3_MIN",
    "REAL_RC3_TRIM",
    "RcLayout",
]
