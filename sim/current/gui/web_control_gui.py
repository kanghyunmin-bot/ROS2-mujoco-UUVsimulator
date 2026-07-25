#!/usr/bin/env python3
"""Compatibility entry point for the MuJoCo UUV web control GUI."""

from __future__ import annotations

import sys
from pathlib import Path


if __package__ in (None, ""):
    _PACKAGE_PARENT = Path(__file__).resolve().parents[1]
    if str(_PACKAGE_PARENT) not in sys.path:
        sys.path.insert(0, str(_PACKAGE_PARENT))
    from gui.web_app import main
else:
    from .web_app import main


if __name__ == "__main__":
    raise SystemExit(main())
