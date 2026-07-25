"""ROS context shutdown error classification."""

from __future__ import annotations


def is_ros_context_shutdown_error(self, exc: Exception) -> bool:
    msg = str(exc).lower()
    return (
        "context is not valid" in msg
        or "context is invalid" in msg
        or "rcl_shutdown" in msg
        or "rcl_init() was not called" in msg
    )


__all__ = ["is_ros_context_shutdown_error"]
