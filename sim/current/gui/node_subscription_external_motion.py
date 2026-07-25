"""External motion and truth subscriptions for the GUI ROS node."""

from __future__ import annotations

from .runtime import Odometry, PoseStamped, TwistStamped, qos_profile_sensor_data


def initialize_external_motion_subscriptions(self) -> None:
    self.create_subscription(
        Odometry,
        "/sim/odom",
        lambda msg: self._on_odom(msg, "/sim/odom"),
        qos_profile_sensor_data,
    )
    self.create_subscription(
        Odometry,
        "/odometry/filtered",
        self._on_filtered_odom,
        qos_profile_sensor_data,
    )
    self.create_subscription(Odometry, "/rovio/odometry", self._on_rovio_odom, qos_profile_sensor_data)
    self.create_subscription(Odometry, "/dvl/odometry", self._on_dvl_odom, qos_profile_sensor_data)
    self.create_subscription(TwistStamped, "/dvl/velocity", self._on_dvl_velocity, qos_profile_sensor_data)
    self.create_subscription(PoseStamped, "/mujoco/ground_truth/pose", self._on_ground_truth_pose, qos_profile_sensor_data)


__all__ = ["initialize_external_motion_subscriptions"]
