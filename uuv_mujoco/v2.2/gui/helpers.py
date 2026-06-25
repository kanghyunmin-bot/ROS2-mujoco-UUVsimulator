"""Compatibility facade for GUI math, RC, backend, and rosbag helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from .backend_helpers import normalize_backend_name
from .config import *  # noqa: F401,F403 - legacy helpers import surface
from .gui_math_helpers import (
    clamp,
    clamp_axis,
    format_age,
    format_replay_time,
    normalize_axes,
    quaternion_to_euler_deg,
    severity_name,
)
from .gui_rc_helpers import (
    althold_level_climb_rate_from_rc3_pwm,
    axis_to_pwm,
    effective_js_gain,
    gui_rc_to_override_axes,
    heave_axis_to_rc3_pwm,
    make_rc_override_message,
    make_rc_release_message,
    manual_heave_axis_to_rc3_pwm,
    padded_rc_channels,
    pilot_heave_axis_summary,
    sanitize_primary_rc_override_channels,
    valid_rc_pwm,
)
from .models import RcReplaySample
from .rc_replay_loader import load_rc_override_replay, rc_replay_bag_uri
from .runtime import (  # noqa: F401 - legacy helpers import surface
    HAVE_MAVROS_MSGS,
    HAVE_ROSBAG2_PY,
    OverrideRCIn,
    StatusText,
    deserialize_message,
    rosbag2_py,
)
