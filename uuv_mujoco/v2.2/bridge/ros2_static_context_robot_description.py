"""Low-rate `/robot_description` publish policy for the ROS2 bridge."""

from __future__ import annotations

from typing import Any, Callable


def robot_description_due(
    *,
    robot_description_pub: Any,
    robot_description_text: str,
    next_publish_t: float,
    sim_t: float,
) -> bool:
    if robot_description_pub is None or not robot_description_text:
        return False
    return float(sim_t) + 1.0e-9 >= float(next_publish_t)


def publish_robot_description_if_due(
    *,
    robot_description_pub: Any,
    string_factory: Callable[[], Any],
    safe_publish: Callable[[Any, Any, str], bool],
    robot_description_text: str,
    next_publish_t: float,
    publish_period_s: float,
    sim_t: float,
) -> tuple[bool, float]:
    """Publish robot description when due and return `(success, next_t)`."""
    if not robot_description_due(
        robot_description_pub=robot_description_pub,
        robot_description_text=robot_description_text,
        next_publish_t=next_publish_t,
        sim_t=sim_t,
    ):
        return True, float(next_publish_t)
    robot_msg = string_factory()
    robot_msg.data = robot_description_text
    if not safe_publish(robot_description_pub, robot_msg, "/robot_description"):
        return False, float(next_publish_t)
    return True, float(sim_t) + float(publish_period_s)


__all__ = ["publish_robot_description_if_due", "robot_description_due"]
