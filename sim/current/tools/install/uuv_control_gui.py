#!/usr/bin/env python3
"""Compatibility entry point for the MuJoCo UUV control GUI.

The GUI implementation lives under the active ``sim/current`` runtime.
This wrapper preserves older commands such as
``python3 uuv_control_gui.py`` and imports from ``uuv_control_gui``.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path
from types import ModuleType


_ROOT = Path(__file__).resolve().parent


def _workspace_root() -> Path:
    candidate = _ROOT.parents[3]
    if (candidate / "sim" / "current" / "gui" / "uuv_control_gui.py").is_file():
        return candidate
    return _ROOT


def _resolve_impl_path() -> Path:
    explicit_runtime = os.environ.get("UUV_MUJOCO_RUNTIME_DIR")
    if explicit_runtime:
        runtime_path = Path(explicit_runtime).expanduser()
        return runtime_path / "gui" / "uuv_control_gui.py"
    return _workspace_root() / "sim" / "current" / "gui" / "uuv_control_gui.py"


_IMPL_PATH = _resolve_impl_path()


def _runtime_root() -> Path:
    return _IMPL_PATH.parents[1]


def _run_freshness_preflight() -> None:
    if os.environ.get("UUV_MUJOCO_SKIP_FRESHNESS_CHECK") == "1":
        return
    runtime_root = _runtime_root()
    checker = runtime_root / "tools" / "check_runtime_freshness.py"
    if not checker.exists():
        print(f"[gui] warning: runtime freshness checker missing: {checker}", file=sys.stderr)
        return
    subprocess.run(
        [
            sys.executable,
            str(checker),
            "--workspace",
            str(_ROOT),
            "--runtime-dir",
            str(runtime_root),
            "--fetch",
            "--refresh-version",
            "--warn-only",
        ],
        check=False,
    )


def _load_impl() -> ModuleType:
    spec = importlib.util.spec_from_file_location("_uuv_control_gui_impl", _IMPL_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load GUI implementation: {_IMPL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


if __name__ == "__main__":
    _run_freshness_preflight()

_impl = _load_impl()
__all__ = [name for name in vars(_impl) if not name.startswith("_")]
globals().update({name: getattr(_impl, name) for name in __all__})


def main() -> int:
    return _impl.main()


if __name__ == "__main__":
    raise SystemExit(main())
