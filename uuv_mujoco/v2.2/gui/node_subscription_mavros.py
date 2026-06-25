"""MAVROS subscriptions for the GUI ROS node."""

from __future__ import annotations

from .runtime import (
    FluidPressure,
    HAVE_MAVROS_MSGS,
    Odometry,
    PoseStamped,
    RCIn,
    RCOut,
    State,
    StatusText,
    TwistStamped,
    qos_profile_sensor_data,
)


def initialize_mavros_navigation_subscriptions(self, *, state_qos) -> None:
    if not HAVE_MAVROS_MSGS:
        return
    self.create_subscription(State, self._topic("state"), self._on_state, state_qos)
    self.create_subscription(PoseStamped, self._topic("local_position/pose"), self._on_pose, qos_profile_sensor_data)
    self.create_subscription(
        Odometry, self._topic("local_position/odom"), self._on_local_odom, qos_profile_sensor_data
    )
    self.create_subscription(
        TwistStamped,
        self._topic("local_position/velocity_body"),
        self._on_velocity_body,
        qos_profile_sensor_data,
    )
    self.create_subscription(
        TwistStamped,
        self._topic("local_position/velocity_local"),
        self._on_velocity_local,
        qos_profile_sensor_data,
    )


def initialize_mavros_rc_status_subscriptions(self, *, best_effort_qos) -> None:
    if not HAVE_MAVROS_MSGS:
        return
    self.create_subscription(RCOut, self._topic("rc/out"), self._on_rc_out, 1)
    self.create_subscription(RCIn, self._topic("rc/in"), self._on_rc_in, 1)
    self.create_subscription(StatusText, self._topic("statustext/recv"), self._on_status_text, best_effort_qos)


def initialize_mavros_pressure_subscriptions(self) -> None:
    if not HAVE_MAVROS_MSGS:
        return
    self.create_subscription(
        FluidPressure,
        self._topic("imu/atm_pressure"),
        self._on_atm_pressure,
        qos_profile_sensor_data,
    )
    self.create_subscription(
        FluidPressure,
        self._topic("imu/static_pressure"),
        self._on_static_pressure,
        qos_profile_sensor_data,
    )


__all__ = [
    "initialize_mavros_navigation_subscriptions",
    "initialize_mavros_pressure_subscriptions",
    "initialize_mavros_rc_status_subscriptions",
]
