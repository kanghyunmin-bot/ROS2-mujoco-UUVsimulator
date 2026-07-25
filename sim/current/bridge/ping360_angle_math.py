"""Ping360 angle conversion helpers."""

from __future__ import annotations

import math

from .ping360_types import PING360_GRADS_PER_REV


def angle_grad_to_rad(angle_grad: int) -> float:
    return 2.0 * math.pi * float(angle_grad) / PING360_GRADS_PER_REV


__all__ = ["angle_grad_to_rad"]
