"""Compatibility exports for MuJoCo viewer launcher/display checks."""

from __future__ import annotations

from dev_os_compat_display import check_display
from dev_os_compat_mjpython import check_mjpython


__all__ = ["check_display", "check_mjpython"]
