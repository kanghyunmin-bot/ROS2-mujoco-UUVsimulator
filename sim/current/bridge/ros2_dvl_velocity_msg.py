"""DVL velocity/range compatibility message builder."""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any

import numpy as np

from .ros2_dvl_header import stamp_header
from .ros2_math import set_first_attr, set_nested_xyz


def _set_dvl_velocity_fields(msg: Any, vel_dvl_frd: np.ndarray) -> None:
    set_nested_xyz(msg, "velocity", vel_dvl_frd)
    set_nested_xyz(msg, "vel", vel_dvl_frd)
    set_first_attr(msg, ("velocity_x", "vx", "surge_velocity"), float(vel_dvl_frd[0]))
    set_first_attr(msg, ("velocity_y", "vy", "sway_velocity"), float(vel_dvl_frd[1]))
    set_first_attr(msg, ("velocity_z", "vz", "heave_velocity"), float(vel_dvl_frd[2]))


def _set_dvl_altitude_fields(msg: Any, altitude_m: float | None) -> None:
    if altitude_m is not None and np.isfinite(altitude_m):
        set_first_attr(msg, ("altitude", "range", "height"), float(altitude_m))


def _new_dvl_beam(parent_msg: Any) -> Any:
    parent_module = type(parent_msg).__module__
    if parent_module.startswith("auv_dvl_a50_msg."):
        try:
            from auv_dvl_a50_msg.msg import DVLBeam

            return DVLBeam()
        except ImportError:
            pass
    try:
        from dvl_msgs.msg import DVLBeam

        return DVLBeam()
    except ImportError:
        # Keeps the message-builder smoke test independent of a sourced ROS
        # workspace.
        return SimpleNamespace(
            id=0,
            velocity=0.0,
            distance=0.0,
            rssi=0.0,
            nsd=0.0,
            valid=False,
        )


def _set_dvl_beams(
    msg: Any,
    vel_dvl_frd: np.ndarray | None,
    altitude_m: float | None,
    *,
    nsd: float,
) -> None:
    """Populate the four valid A50 beams required by the physical bridge."""

    if not hasattr(msg, "beams"):
        return
    velocity = None if vel_dvl_frd is None else np.asarray(vel_dvl_frd, dtype=float)
    valid = bool(
        velocity is not None
        and velocity.shape == (3,)
        and np.all(np.isfinite(velocity))
        and altitude_m is not None
        and math.isfinite(float(altitude_m))
        and float(altitude_m) > 0.0
    )
    # Water Linked A50 uses four slanted bottom-track beams. The bridge only
    # gates on validity, but physically plausible projections make rosbag and
    # downstream diagnostics obey the real message contract too.
    tilt_rad = math.radians(22.5)
    distance_m = (
        float(altitude_m) / math.cos(tilt_rad) if valid else 0.0
    )
    beams = []
    for beam_id, azimuth_deg in enumerate((45.0, 135.0, 225.0, 315.0)):
        azimuth_rad = math.radians(azimuth_deg)
        direction = np.array(
            [
                math.sin(tilt_rad) * math.cos(azimuth_rad),
                math.sin(tilt_rad) * math.sin(azimuth_rad),
                math.cos(tilt_rad),
            ],
            dtype=float,
        )
        beam = _new_dvl_beam(msg)
        beam.id = int(beam_id)
        beam.velocity = float(np.dot(velocity, direction)) if valid else 0.0
        beam.distance = distance_m
        beam.rssi = 80.0 if valid else 0.0
        beam.nsd = float(max(nsd, 0.0))
        beam.valid = valid
        beams.append(beam)
    msg.beams = beams


def build_dvl_msg(
    dvl_msg_type: type | None,
    stamp: Any,
    vel_dvl_frd: np.ndarray | None,
    altitude_m: float | None,
    *,
    sample_period_s: float = 0.02,
    covariance: tuple[float, ...] = (
        4.696386440627975e-6, 0.0, 0.0,
        0.0, 1.173283067146258e-6, 0.0,
        0.0, 0.0, 1.566586860235475e-7,
    ),
) -> Any | None:
    if dvl_msg_type is None:
        return None
    msg = dvl_msg_type()
    stamp_header(msg, stamp, "dvl")
    if vel_dvl_frd is not None:
        _set_dvl_velocity_fields(msg, vel_dvl_frd)
    _set_dvl_altitude_fields(msg, altitude_m)
    if hasattr(msg, "time"):
        msg.time = float(max(sample_period_s, 0.0) * 1000.0)
    if hasattr(msg, "covariance"):
        msg.covariance = [float(value) for value in covariance]
    if hasattr(msg, "fom"):
        msg.fom = float(np.sqrt(max(covariance[0], covariance[4], covariance[8], 0.0)))
    fom = float(np.sqrt(max(covariance[0], covariance[4], covariance[8], 0.0)))
    _set_dvl_beams(msg, vel_dvl_frd, altitude_m, nsd=fom)
    if hasattr(msg, "velocity_valid"):
        msg.velocity_valid = bool(
            vel_dvl_frd is not None
            and np.asarray(vel_dvl_frd).shape == (3,)
            and np.all(np.isfinite(vel_dvl_frd))
            and altitude_m is not None
            and np.isfinite(altitude_m)
            and float(altitude_m) > 0.0
        )
    if hasattr(msg, "status"):
        msg.status = 0
    stamp_us = _stamp_to_microseconds(stamp)
    if hasattr(msg, "time_of_validity"):
        msg.time_of_validity = stamp_us
    if hasattr(msg, "time_of_transmission"):
        msg.time_of_transmission = stamp_us
    if hasattr(msg, "form"):
        msg.form = "simulated_a50_velocity"
    return msg


def _stamp_to_microseconds(stamp: Any) -> int:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return 0
    return int(sec) * 1_000_000 + int(nanosec) // 1_000


__all__ = ["build_dvl_msg"]
