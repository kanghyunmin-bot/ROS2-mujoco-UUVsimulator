"""Math, RC, backend, and rosbag helper functions for the GUI."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

from .config import *
from .models import RcReplaySample
from .runtime import (
    HAVE_MAVROS_MSGS,
    HAVE_ROSBAG2_PY,
    OverrideRCIn,
    StatusText,
    deserialize_message,
    rosbag2_py,
)

def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_axis(value: float) -> float:
    return clamp(float(value), AXIS_MIN, AXIS_MAX)


def normalize_axes(
    *,
    forward: float,
    lateral: float,
    heave: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    return (
        clamp_axis(forward),
        clamp_axis(lateral),
        clamp_axis(heave),
        clamp_axis(yaw),
    )


def format_age(age_s: float) -> str:
    if not math.isfinite(age_s):
        return "n/a"
    if age_s < 1.0:
        return f"{age_s * 1000.0:.0f} ms"
    return f"{age_s:.1f} s"


def format_replay_time(time_s: float) -> str:
    if not math.isfinite(time_s):
        return "--:--"
    time_s = max(0.0, float(time_s))
    minutes = int(time_s // 60.0)
    seconds = time_s - minutes * 60.0
    return f"{minutes:02d}:{seconds:04.1f}"


def severity_name(level: int) -> str:
    names = {
        StatusText.EMERGENCY: "EMERGENCY",
        StatusText.ALERT: "ALERT",
        StatusText.CRITICAL: "CRITICAL",
        StatusText.ERROR: "ERROR",
        StatusText.WARNING: "WARNING",
        StatusText.NOTICE: "NOTICE",
        StatusText.INFO: "INFO",
        StatusText.DEBUG: "DEBUG",
    }
    return names.get(level, f"S{level}")


def quaternion_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def axis_to_pwm(value: float) -> int:
    """Map a normalized joystick axis in [-1, 1] to ArduSub RC PWM."""
    return int(round(RC_NEUTRAL_PWM + clamp_axis(value) * RC_PWM_SPAN))


def effective_js_gain(
    *,
    gain_default: float = REAL_JS_GAIN_DEFAULT,
    gain_min: float = REAL_JS_GAIN_MIN,
    gain_max: float = REAL_JS_GAIN_MAX,
    gain_steps: int = REAL_JS_GAIN_STEPS,
) -> float:
    """Mirror ArduSub joystick.cpp init_joystick() gain selection."""
    steps = max(1, int(gain_steps))
    default = float(gain_default)
    min_gain = float(gain_min)
    max_gain = float(gain_max)
    if steps == 1 or (default < max_gain + 0.01 and default > min_gain - 0.01):
        gain = clamp(default, min_gain, max_gain)
    else:
        gain = min_gain + (steps / 2.0 - 1.0) * (max_gain - min_gain) / float(steps - 1)
    return clamp(gain, 0.1, 1.0)


def manual_heave_axis_to_rc3_pwm(
    value: float,
    *,
    gain: float | None = None,
    throttle_gain: float = REAL_JS_THR_GAIN,
) -> int:
    """Mirror ArduSub MANUAL_CONTROL.z -> RC3 conversion for GUI diagnostics."""
    gain = effective_js_gain() if gain is None else clamp(float(gain), 0.1, 1.0)
    throttle_scale = 0.8 * gain * float(throttle_gain)
    throttle_base = RC_NEUTRAL_PWM - 500.0 * throttle_scale
    manual_z = 500.0 + clamp_axis(value) * 500.0
    return int(round(clamp(manual_z * throttle_scale + throttle_base, REAL_RC3_MIN, REAL_RC3_MAX)))


def althold_level_climb_rate_from_rc3_pwm(
    rc3_pwm: float,
    *,
    gain: float | None = None,
    rc_min: int = REAL_RC3_MIN,
    rc_max: int = REAL_RC3_MAX,
    rc_trim: int = REAL_RC3_TRIM,
    rc_deadzone: int = REAL_RC3_DZ,
    pilot_speed_up: float = REAL_PILOT_SPEED_UP,
    pilot_speed_dn: float = REAL_PILOT_SPEED_DN,
) -> float:
    """Approximate ArduSub ALT_HOLD target climb rate for a level vehicle."""
    rc3_pwm = float(rc3_pwm)
    if rc3_pwm < rc_trim:
        norm = 0.0 if rc_min >= rc_trim else (rc3_pwm - rc_trim) / float(rc_trim - rc_min)
    else:
        norm = 0.0 if rc_max <= rc_trim else (rc3_pwm - rc_trim) / float(rc_max - rc_trim)
    norm = clamp(norm, -1.0, 1.0)
    earth_z = 2.0 * (-0.5 + norm)
    throttle_control = 500.0 + float(pilot_speed_up) * earth_z
    center = (float(rc_max) + float(rc_min)) / 2.0
    target_climb = throttle_control - center + 1000.0
    gain = effective_js_gain() if gain is None else clamp(float(gain), 0.1, 1.0)
    if abs(target_climb) < float(rc_deadzone) * gain:
        target_climb = 0.0
    speed_down = abs(float(pilot_speed_dn)) if float(pilot_speed_dn) != 0.0 else abs(float(pilot_speed_up))
    return clamp(target_climb, -speed_down, float(pilot_speed_up))


def pilot_heave_axis_summary(value: float, *, mode: str = GUI_PILOT_CONTROL_MODE) -> tuple[int, float]:
    """Return expected RC3 PWM and level ALT_HOLD climb target for GUI stick heave."""
    if mode == PILOT_CONTROL_RC_OVERRIDE:
        rc3_pwm = axis_to_pwm(value)
    else:
        rc3_pwm = manual_heave_axis_to_rc3_pwm(value)
    return rc3_pwm, althold_level_climb_rate_from_rc3_pwm(rc3_pwm)


def gui_rc_to_override_axes(
    *,
    forward: float,
    lateral: float,
    heave: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Map GUI stick axes directly to RC override axes."""
    return normalize_axes(
        forward=forward,
        lateral=lateral,
        heave=heave,
        yaw=yaw,
    )


def make_rc_override_message(
    layout: RcLayout,
    *,
    yaw: float,
    heave: float,
    forward: float,
    lateral: float,
) -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
    for idx in range(PRIMARY_RC_CHANNEL_COUNT):
        msg.channels[idx] = RC_NEUTRAL_PWM
    axes = {
        "heave": heave,
        "yaw": yaw,
        "forward": forward,
        "lateral": lateral,
    }
    for axis_name, axis_value in axes.items():
        msg.channels[int(layout.axis_channels[axis_name])] = axis_to_pwm(axis_value)
    return msg


def make_rc_release_message() -> OverrideRCIn:
    msg = OverrideRCIn()
    msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
    for idx in range(PRIMARY_RC_CHANNEL_COUNT):
        msg.channels[idx] = OverrideRCIn.CHAN_RELEASE
    return msg


def padded_rc_channels(
    values: Iterable[int],
    *,
    target_count: int = RC_FEEDBACK_CHANNEL_COUNT,
    sanitize_override_markers: bool = False,
) -> list[int]:
    channels: list[int] = []
    for value in list(values)[:target_count]:
        ivalue = int(value)
        if sanitize_override_markers and ivalue in (OverrideRCIn.CHAN_NOCHANGE, OverrideRCIn.CHAN_RELEASE):
            ivalue = 0
        elif sanitize_override_markers and (ivalue < 800 or ivalue > 2200):
            ivalue = 0
        channels.append(ivalue)
    if len(channels) < target_count:
        channels.extend([0] * (target_count - len(channels)))
    return channels


def rc_replay_bag_uri(path_text: str) -> str:
    path = Path(path_text).expanduser()
    if path.suffix == ".db3":
        return str(path.parent)
    return str(path)


def load_rc_override_replay(path_text: str) -> list[RcReplaySample]:
    if not HAVE_MAVROS_MSGS:
        raise RuntimeError("mavros_msgs is not available in this Python environment")
    if not HAVE_ROSBAG2_PY or rosbag2_py is None or deserialize_message is None:
        raise RuntimeError("rosbag2_py is not available in this Python environment")

    bag_uri = rc_replay_bag_uri(path_text)
    if not Path(bag_uri).exists():
        raise RuntimeError(f"bag path does not exist: {bag_uri}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if RC_REPLAY_TOPIC not in topics:
        raise RuntimeError(f"{RC_REPLAY_TOPIC} not found in bag")
    if topics[RC_REPLAY_TOPIC] != "mavros_msgs/msg/OverrideRCIn":
        raise RuntimeError(f"{RC_REPLAY_TOPIC} has unexpected type: {topics[RC_REPLAY_TOPIC]}")

    samples: list[RcReplaySample] = []
    first_timestamp_ns: int | None = None
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != RC_REPLAY_TOPIC:
            continue
        if first_timestamp_ns is None:
            first_timestamp_ns = int(timestamp_ns)
        msg = deserialize_message(data, OverrideRCIn)
        channels = padded_rc_channels(
            getattr(msg, "channels", []),
            target_count=RC_MESSAGE_CHANNEL_COUNT,
            sanitize_override_markers=False,
        )
        samples.append(
            RcReplaySample(
                time_s=(int(timestamp_ns) - first_timestamp_ns) * 1e-9,
                channels=tuple(channels[:RC_MESSAGE_CHANNEL_COUNT]),
            )
        )

    if not samples:
        raise RuntimeError(f"{RC_REPLAY_TOPIC} has no messages")
    return samples


def normalize_backend_name(name: str) -> str:
    value = str(name or BACKEND_AUTO).strip().lower()
    if value in ("none", "off", BACKEND_NONE):
        return BACKEND_NONE
    if value in ("sim", BACKEND_SIM_BRIDGE):
        return BACKEND_SIM_BRIDGE
    if value == BACKEND_MAVROS:
        return BACKEND_MAVROS
    return BACKEND_AUTO
