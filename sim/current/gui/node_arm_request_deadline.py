"""Deadline helpers for GUI arm/disarm requests."""

from __future__ import annotations

import time
from typing import Optional


def arm_deadline(self, deadline: Optional[float]) -> float:
    if deadline is not None:
        return float(deadline)
    return time.monotonic() + self._control_request_timeout_s


__all__ = ["arm_deadline"]
