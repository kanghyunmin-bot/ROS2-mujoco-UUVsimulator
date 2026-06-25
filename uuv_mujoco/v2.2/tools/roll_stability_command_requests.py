"""MAVROS request builders for the roll-stability probe."""

from __future__ import annotations

from mavros_msgs.srv import CommandBool, SetMode


def build_set_mode_request(mode: str) -> SetMode.Request:
    req = SetMode.Request()
    req.base_mode = 0
    req.custom_mode = mode
    return req


def build_arm_request(value: bool) -> CommandBool.Request:
    req = CommandBool.Request()
    req.value = bool(value)
    return req


__all__ = ["build_arm_request", "build_set_mode_request"]
