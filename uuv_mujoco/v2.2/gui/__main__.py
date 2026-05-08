#!/usr/bin/env python3
"""Run the MuJoCo UUV control GUI package."""

from __future__ import annotations

try:
    from .uuv_control_gui import main
except ImportError:
    from uuv_control_gui import main


if __name__ == "__main__":
    raise SystemExit(main())
