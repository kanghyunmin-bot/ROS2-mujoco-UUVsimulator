#!/usr/bin/env python3
"""Compatibility module for the MuJoCo UUV control GUI implementation."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def _freshness_runtime_dir(runtime_root: Path, workspace_root: Path) -> Path:
    env_runtime = os.environ.get("UUV_MUJOCO_RUNTIME_DIR")
    if env_runtime:
        return Path(env_runtime)

    active_alias = workspace_root / "uuv_mujoco" / "current"
    try:
        if active_alias.exists() and active_alias.resolve() == runtime_root.resolve():
            return active_alias
    except OSError:
        pass
    return runtime_root


def _run_direct_freshness_preflight() -> None:
    if os.environ.get("UUV_MUJOCO_SKIP_FRESHNESS_CHECK") == "1":
        return
    runtime_root = Path(__file__).resolve().parents[1]
    workspace_root = Path(__file__).resolve().parents[3]
    freshness_runtime = _freshness_runtime_dir(runtime_root, workspace_root)
    checker = runtime_root / "tools" / "check_runtime_freshness.py"
    if not checker.exists():
        print(f"[gui] warning: runtime freshness checker missing: {checker}", file=sys.stderr)
        return
    subprocess.run(
        [
            sys.executable,
            str(checker),
            "--workspace",
            str(workspace_root),
            "--runtime-dir",
            str(freshness_runtime),
            "--fetch",
            "--refresh-version",
            "--warn-only",
        ],
        check=False,
    )


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
    _run_direct_freshness_preflight()
    raise SystemExit(main())
