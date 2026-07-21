"""Publish helpers for axis RC validation nodes."""

from __future__ import annotations

from axis_rc_messages import (
    build_manual_control_message,
    build_rc_override_message,
    build_rc_release_message,
)


def publish_rc_override(node, axis: str | None = None, command: float = 0.0) -> None:
    msg = build_rc_override_message(axis, command, invert_heave_rc=node.invert_heave_rc)
    node._rc_released = False
    node.rc_pub.publish(msg)


def publish_manual_control(node, axis: str | None = None, command: float = 0.0) -> None:
    node.manual_pub.publish(build_manual_control_message(axis, command))


def publish_rc_release(node) -> None:
    node.rc_pub.publish(build_rc_release_message())
    node._rc_released = True


def publish_neutral_control(node) -> None:
    if node.input_mode == "manual-control":
        if not node._rc_released:
            publish_rc_release(node)
        publish_manual_control(node)
    elif node.input_mode == "both":
        publish_rc_override(node)
        publish_manual_control(node)
    else:
        publish_rc_override(node)


__all__ = [
    "publish_manual_control",
    "publish_neutral_control",
    "publish_rc_override",
    "publish_rc_release",
]
