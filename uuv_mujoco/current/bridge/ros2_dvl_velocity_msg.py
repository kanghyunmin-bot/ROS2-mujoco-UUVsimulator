"""DVL velocity/range compatibility message builder."""

from __future__ import annotations

import importlib
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


def _new_dvl_beam(msg: Any) -> Any:
    package_candidates = [type(msg).__module__.split(".", maxsplit=1)[0]]
    package_candidates.extend(("auv_dvl_a50_msg", "dvl_msgs"))
    for package_name in dict.fromkeys(package_candidates):
        try:
            message_module = importlib.import_module(f"{package_name}.msg")
            return message_module.DVLBeam()
        except (ImportError, AttributeError):
            continue
    # Keeps the message-builder smoke test independent of a sourced ROS
    # workspace. A sourced physical stack resolves one of the packages above.
    return SimpleNamespace(
        id=0,
        velocity=0.0,
        distance=-1.0,
        rssi=-120.0,
        nsd=-94.0,
        valid=False,
    )


def _set_dvl_beams(
    msg: Any,
    vel_dvl_frd: np.ndarray | None,
    altitude_m: float | None,
    *,
    nsd: float,
    sensor_sample: Any | None = None,
) -> None:
    """Populate A50 transducer reports, preserving dBm diagnostic units."""

    if not hasattr(msg, "beams"):
        return
    if sensor_sample is not None:
        beams = []
        for source in sensor_sample.beams:
            beam = _new_dvl_beam(msg)
            beam.id = int(source.beam_id)
            beam.velocity = float(source.measured_radial_velocity_mps or 0.0)
            beam.distance = (
                float(source.measured_range_m)
                if source.valid and source.measured_range_m is not None
                else -1.0
            )
            beam.rssi = float(source.rssi_dbm)
            beam.nsd = float(source.nsd_dbm)
            beam.valid = bool(source.valid)
            beams.append(beam)
        msg.beams = beams
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
        beam.rssi = -40.0 if valid else -120.0
        beam.nsd = -94.0
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
    sensor_sample: Any | None = None,
) -> Any | None:
    if dvl_msg_type is None:
        return None
    msg = dvl_msg_type()
    stamp_header(msg, stamp, "dvl_link")
    if sensor_sample is not None:
        vel_dvl_frd = (
            None
            if sensor_sample.measured_velocity_frd_mps is None
            else np.asarray(sensor_sample.measured_velocity_frd_mps, dtype=np.float64)
        )
        altitude_m = sensor_sample.altitude_estimate_m
        if sensor_sample.covariance_frd_mps2 is not None:
            covariance = tuple(float(value) for value in sensor_sample.covariance_frd_mps2)
    if vel_dvl_frd is not None:
        _set_dvl_velocity_fields(msg, vel_dvl_frd)
    _set_dvl_altitude_fields(msg, altitude_m)
    if sensor_sample is not None and altitude_m is None and hasattr(msg, "altitude"):
        msg.altitude = -1.0
    if hasattr(msg, "time"):
        msg.time = float(max(sample_period_s, 0.0) * 1000.0)
    if hasattr(msg, "covariance"):
        msg.covariance = [float(value) for value in covariance]
    covariance_fom = float(np.sqrt(max(covariance[0], covariance[4], covariance[8], 0.0)))
    fom = (
        float(sensor_sample.fom_mps)
        if sensor_sample is not None and math.isfinite(float(sensor_sample.fom_mps))
        else (2.707 if sensor_sample is not None else covariance_fom)
    )
    if hasattr(msg, "fom"):
        msg.fom = fom
    _set_dvl_beams(
        msg,
        vel_dvl_frd,
        altitude_m,
        nsd=covariance_fom,
        sensor_sample=sensor_sample,
    )
    if hasattr(msg, "velocity_valid"):
        msg.velocity_valid = (
            bool(sensor_sample.velocity_valid)
            if sensor_sample is not None
            else bool(
                vel_dvl_frd is not None
                and np.asarray(vel_dvl_frd).shape == (3,)
                and np.all(np.isfinite(vel_dvl_frd))
                and altitude_m is not None
                and np.isfinite(altitude_m)
                and float(altitude_m) > 0.0
            )
        )
    if hasattr(msg, "status"):
        msg.status = 0
    stamp_us = _stamp_to_microseconds(stamp)
    if hasattr(msg, "time_of_validity"):
        msg.time_of_validity = (
            int(sensor_sample.time_of_validity_us)
            if sensor_sample is not None
            else stamp_us
        )
    if hasattr(msg, "time_of_transmission"):
        msg.time_of_transmission = (
            int(sensor_sample.time_of_transmission_us)
            if sensor_sample is not None
            else stamp_us
        )
    if hasattr(msg, "form"):
        msg.form = "json_v3.3" if sensor_sample is not None else "simulated_a50_velocity"
    return msg


def _stamp_to_microseconds(stamp: Any) -> int:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return 0
    return int(sec) * 1_000_000 + int(nanosec) // 1_000


__all__ = ["build_dvl_msg"]
