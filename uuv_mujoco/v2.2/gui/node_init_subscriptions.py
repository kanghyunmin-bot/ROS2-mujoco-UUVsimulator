"""Subscription initialization for the GUI ROS node."""

from __future__ import annotations

from .node_subscription_core_sensors import initialize_core_sensor_subscriptions
from .node_subscription_depth import initialize_depth_subscriptions
from .node_subscription_external_motion import initialize_external_motion_subscriptions
from .node_subscription_mavros import (
    initialize_mavros_navigation_subscriptions,
    initialize_mavros_pressure_subscriptions,
    initialize_mavros_rc_status_subscriptions,
)
from .node_subscription_qos import build_best_effort_qos, build_state_qos
from .node_subscription_status import initialize_status_subscriptions


def initialize_subscriptions(self) -> None:
    state_qos = build_state_qos()
    best_effort_qos = build_best_effort_qos()
    initialize_status_subscriptions(self)
    initialize_core_sensor_subscriptions(self, best_effort_qos=best_effort_qos)
    initialize_mavros_navigation_subscriptions(self, state_qos=state_qos)
    initialize_external_motion_subscriptions(self)
    initialize_mavros_rc_status_subscriptions(self, best_effort_qos=best_effort_qos)
    initialize_depth_subscriptions(self, best_effort_qos=best_effort_qos)
    initialize_mavros_pressure_subscriptions(self)


__all__ = ["initialize_subscriptions"]
