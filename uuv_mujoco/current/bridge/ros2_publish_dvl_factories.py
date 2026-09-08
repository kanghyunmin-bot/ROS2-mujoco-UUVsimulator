"""DVL ROS message factory functions for publish jobs."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from .dvl_a50_tcp_emulator import INVALID_FOM_MPS
from .ros2_bridge_publish_stamp import stamp_from_seconds_like
from .ros2_dvl_messages import build_dvl_msg, build_dvldr_msg
from .ros2_standard_messages import build_range_msg, build_twist_cov_msg, build_twist_msg

DVL_LINEAR_COV_DIAG = (4.696386440627975e-6, 1.173283067146258e-6, 1.566586860235475e-7)
DVL_ANGULAR_COV_DIAG = (0.0, 0.0, 0.0)
DvlFactory = Callable[[object, object, object], object]


def _sensor_delivery(bridge, state):
    if not bool(getattr(bridge, "_dvl_sensor_model_enabled", False)):
        return None
    return state.dvl_sensor_delivery


def _sensor_sample(bridge, state):
    delivery = _sensor_delivery(bridge, state)
    return None if delivery is None else delivery.sample


def _position_delivery(bridge, state):
    if not bool(getattr(bridge, "_dvl_sensor_model_enabled", False)):
        return None
    return state.dvl_position_delivery


def _capture_stamp(bridge, stamp, state):
    delivery = _sensor_delivery(bridge, state)
    if delivery is None:
        return stamp
    return stamp_from_seconds_like(stamp, delivery.capture_time_s)


def _dvl_velocity_frd(bridge, state) -> np.ndarray | None:
    if bool(getattr(bridge, "_dvl_sensor_model_enabled", False)):
        sample = _sensor_sample(bridge, state)
        if sample is None or sample.measured_velocity_frd_mps is None:
            return None
        return np.asarray(sample.measured_velocity_frd_mps, dtype=np.float64)
    return state.dvl_vel_dvl_frd


def _dvl_altitude(bridge, state) -> float | None:
    if bool(getattr(bridge, "_dvl_sensor_model_enabled", False)):
        sample = _sensor_sample(bridge, state)
        return None if sample is None else sample.altitude_estimate_m
    return state.dvl_altitude_m


def _dvl_velocity_body_ros(bridge, state) -> np.ndarray | None:
    velocity_dvl_frd = _dvl_velocity_frd(bridge, state)
    if velocity_dvl_frd is None:
        return None
    velocity_body_frd = bridge._dvl_body_frd_to_dvl_frd.T @ velocity_dvl_frd
    velocity_body_bmj = bridge._bmj_to_frd.T @ velocity_body_frd
    return bridge._bmj_to_flu @ velocity_body_bmj


def _linear_covariance_diag(bridge, state) -> tuple[float, float, float]:
    sample = _sensor_sample(bridge, state)
    if sample is None or sample.covariance_frd_mps2 is None:
        return DVL_LINEAR_COV_DIAG
    covariance = sample.covariance_frd_mps2
    return float(covariance[0]), float(covariance[4]), float(covariance[8])


def build_dvl_velocity_msg(bridge, stamp, state):
    velocity_body_ros = _dvl_velocity_body_ros(bridge, state)
    if velocity_body_ros is None:
        return None
    return build_twist_msg(
        bridge.TwistStamped,
        _capture_stamp(bridge, stamp, state),
        "base_link",
        velocity_body_ros,
    )


def build_dvl_twist_msg(bridge, stamp, state):
    velocity_dvl_frd = _dvl_velocity_frd(bridge, state)
    if velocity_dvl_frd is None:
        return None
    return build_twist_cov_msg(
        bridge.TwistWithCovarianceStamped,
        _capture_stamp(bridge, stamp, state),
        "dvl_link",
        velocity_dvl_frd,
        linear_cov_diag=_linear_covariance_diag(bridge, state),
        angular_cov_diag=DVL_ANGULAR_COV_DIAG,
    )


def build_dvl_altitude_msg(bridge, stamp, state):
    altitude_m = _dvl_altitude(bridge, state)
    if altitude_m is None:
        return None
    return build_range_msg(
        bridge.Range,
        _capture_stamp(bridge, stamp, state),
        float(altitude_m),
    )


def build_dvl_data_msg(bridge, stamp, state):
    delivery = _sensor_delivery(bridge, state)
    sample = _sensor_sample(bridge, state)
    velocity_dvl_frd = _dvl_velocity_frd(bridge, state)
    altitude_m = _dvl_altitude(bridge, state)
    if bool(getattr(bridge, "_dvl_sensor_model_enabled", False)) and sample is None:
        return None
    if sample is not None and sample.covariance_frd_mps2 is None:
        invalid_variance = INVALID_FOM_MPS**2
        covariance = (
            invalid_variance, 0.0, 0.0,
            0.0, invalid_variance, 0.0,
            0.0, 0.0, invalid_variance,
        )
    elif sample is not None:
        covariance = sample.covariance_frd_mps2
    else:
        covariance = (
            DVL_LINEAR_COV_DIAG[0], 0.0, 0.0,
            0.0, DVL_LINEAR_COV_DIAG[1], 0.0,
            0.0, 0.0, DVL_LINEAR_COV_DIAG[2],
        )
    return build_dvl_msg(
        bridge.DVLMsg,
        (
            stamp_from_seconds_like(stamp, delivery.arrival_time_s)
            if delivery is not None
            else stamp
        ),
        velocity_dvl_frd,
        altitude_m,
        sample_period_s=(
            delivery.report_period_s
            if delivery is not None
            else 1.0 / max(float(bridge._ros_rate_dvl_twist_hz), 1.0e-6)
        ),
        covariance=tuple(float(value) for value in covariance),
        sensor_sample=sample,
    )


def build_dvl_position_msg(bridge, stamp, state):
    delivery = _position_delivery(bridge, state)
    if bool(getattr(bridge, "_dvl_sensor_model_enabled", False)):
        if delivery is None:
            return None
        return build_dvldr_msg(
            bridge.DVLDRMsg,
            stamp_from_seconds_like(stamp, delivery.arrival_time_s),
            np.asarray(delivery.position_local_frd_m, dtype=np.float64),
            state.quat_ros,
            attitude_rpy_deg=delivery.attitude_rpy_deg,
            position_std_m=delivery.position_std_m,
            report_time_s=delivery.report_time_s,
        )
    return build_dvldr_msg(bridge.DVLDRMsg, stamp, bridge._odom_pos, state.quat_ros)


DVL_PUBLISH_FACTORIES: tuple[tuple[str, DvlFactory], ...] = (
    ("dvl_velocity", build_dvl_velocity_msg),
    ("dvl_twist", build_dvl_twist_msg),
    ("dvl_altitude", build_dvl_altitude_msg),
    ("dvl_data", build_dvl_data_msg),
    ("dvl_position", build_dvl_position_msg),
)


__all__ = [
    "DVL_ANGULAR_COV_DIAG",
    "DVL_LINEAR_COV_DIAG",
    "DVL_PUBLISH_FACTORIES",
    "DvlFactory",
    "build_dvl_altitude_msg",
    "build_dvl_data_msg",
    "build_dvl_position_msg",
    "build_dvl_twist_msg",
    "build_dvl_velocity_msg",
]
