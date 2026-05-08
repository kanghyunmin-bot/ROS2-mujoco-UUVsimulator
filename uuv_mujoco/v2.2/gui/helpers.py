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
