"""Low-rate static ROS context publishing."""

from __future__ import annotations

from typing import Any, Callable

from .ros2_static_context_robot_description import publish_robot_description_if_due
from .ros2_static_context_tf import publish_static_tf_once


class StaticContextPublisher:
    """Manage low-rate static bridge outputs outside the main hot path."""

    def __init__(
        self,
        *,
        tf_static_pub: Any,
        robot_description_pub: Any,
        string_factory: Callable[[], Any],
        build_tf_message: Callable[[Any, Any], Any],
        safe_publish: Callable[[Any, Any, str], bool],
        static_tf_specs: Any,
        robot_description_text: str,
        robot_description_pub_period_s: float,
    ) -> None:
        self._tf_static_pub = tf_static_pub
        self._robot_description_pub = robot_description_pub
        self._string_factory = string_factory
        self._build_tf_message = build_tf_message
        self._safe_publish = safe_publish
        self._static_tf_specs = static_tf_specs
        self._robot_description_text = robot_description_text
        self._robot_description_pub_period_s = float(robot_description_pub_period_s)
        self._robot_description_next_t = 0.0
        self._static_tf_published = False

    def publish(self, stamp: Any, sim_t: float) -> bool:
        if not self._publish_static_tf(stamp):
            return False
        return self._publish_robot_description(sim_t)

    def _publish_static_tf(self, stamp: Any) -> bool:
        success, published = publish_static_tf_once(
            tf_static_pub=self._tf_static_pub,
            build_tf_message=self._build_tf_message,
            safe_publish=self._safe_publish,
            static_tf_specs=self._static_tf_specs,
            stamp=stamp,
            already_published=self._static_tf_published,
        )
        self._static_tf_published = published
        return success

    def _publish_robot_description(self, sim_t: float) -> bool:
        success, next_t = publish_robot_description_if_due(
            robot_description_pub=self._robot_description_pub,
            string_factory=self._string_factory,
            safe_publish=self._safe_publish,
            robot_description_text=self._robot_description_text,
            next_publish_t=self._robot_description_next_t,
            publish_period_s=self._robot_description_pub_period_s,
            sim_t=sim_t,
        )
        self._robot_description_next_t = next_t
        return success


__all__ = ["StaticContextPublisher"]
