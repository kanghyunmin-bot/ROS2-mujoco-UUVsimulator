"""DVL ROS message factory functions for publish jobs."""

from __future__ import annotations

from collections.abc import Callable

from .ros2_dvl_messages import build_dvl_msg, build_dvldr_msg
from .ros2_standard_messages import build_range_msg, build_twist_cov_msg, build_twist_msg

DVL_LINEAR_COV_DIAG = (4.696386440627975e-6, 1.173283067146258e-6, 1.566586860235475e-7)
DVL_ANGULAR_COV_DIAG = (0.0, 0.0, 0.0)
DvlFactory = Callable[[object, object, object], object]


def build_dvl_velocity_msg(bridge, stamp, state):
    if state.dvl_vel_body_ros is None:
        return None
    return build_twist_msg(bridge.TwistStamped, stamp, "base_link", state.dvl_vel_body_ros)


def build_dvl_twist_msg(bridge, stamp, state):
    if state.dvl_vel_dvl_frd is None:
        return None
    return build_twist_cov_msg(
        bridge.TwistWithCovarianceStamped,
        stamp,
        "dvl",
        state.dvl_vel_dvl_frd,
        linear_cov_diag=DVL_LINEAR_COV_DIAG,
        angular_cov_diag=DVL_ANGULAR_COV_DIAG,
    )


def build_dvl_altitude_msg(bridge, stamp, state):
    if state.dvl_altitude_m is None:
        return None
    return build_range_msg(bridge.Range, stamp, float(state.dvl_altitude_m))


def build_dvl_data_msg(bridge, stamp, state):
    return build_dvl_msg(
        bridge.DVLMsg,
        stamp,
        state.dvl_vel_dvl_frd,
        state.dvl_altitude_m,
        sample_period_s=1.0 / max(float(bridge._ros_rate_dvl_twist_hz), 1.0e-6),
        covariance=(
            DVL_LINEAR_COV_DIAG[0], 0.0, 0.0,
            0.0, DVL_LINEAR_COV_DIAG[1], 0.0,
            0.0, 0.0, DVL_LINEAR_COV_DIAG[2],
        ),
    )


def build_dvl_position_msg(bridge, stamp, state):
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
