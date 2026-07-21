"""Loose ROS message setter helpers for optional message classes."""

from __future__ import annotations


def set_nested_xyz(msg, attr: str, xyz) -> None:
    if not hasattr(msg, attr):
        return
    obj = getattr(msg, attr)
    for axis, value in zip(("x", "y", "z"), xyz.tolist()):
        if hasattr(obj, axis):
            setattr(obj, axis, float(value))


def set_first_attr(msg, names, value) -> None:
    for name in names:
        if hasattr(msg, name):
            setattr(msg, name, value)
            return
