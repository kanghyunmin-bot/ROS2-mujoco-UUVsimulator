#!/usr/bin/env python3
"""Compatibility module for the MuJoCo UUV control GUI implementation."""

from __future__ import annotations

import sys
from pathlib import Path

if __package__ in (None, ""):
    _PACKAGE_PARENT = Path(__file__).resolve().parents[1]
    if str(_PACKAGE_PARENT) not in sys.path:
        sys.path.insert(0, str(_PACKAGE_PARENT))
    from gui.config import *  # type: ignore[F401,F403]
    from gui.helpers import *  # type: ignore[F401,F403]
    from gui.models import *  # type: ignore[F401,F403]
    from gui.node import *  # type: ignore[F401,F403]
    from gui.ros_tools import *  # type: ignore[F401,F403]
    from gui.runtime import *  # type: ignore[F401,F403]
    from gui.widgets import *  # type: ignore[F401,F403]
    from gui.app import *  # type: ignore[F401,F403]
    from gui.app import main
else:
    from .config import *  # type: ignore[F401,F403]
    from .helpers import *  # type: ignore[F401,F403]
    from .models import *  # type: ignore[F401,F403]
    from .node import *  # type: ignore[F401,F403]
    from .ros_tools import *  # type: ignore[F401,F403]
    from .runtime import *  # type: ignore[F401,F403]
    from .widgets import *  # type: ignore[F401,F403]
    from .app import *  # type: ignore[F401,F403]
    from .app import main


if __name__ == "__main__":
    raise SystemExit(main())
