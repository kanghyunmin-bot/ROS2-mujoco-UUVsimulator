#!/usr/bin/env python3
"""Compatibility entry point for the MuJoCo UUV control GUI.

The GUI implementation lives under ``uuv_mujoco/v2.2/gui`` so it stays with
the MuJoCo runtime. This wrapper preserves older commands such as
``python3 uuv_control_gui.py`` and imports from ``uuv_control_gui``.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType


_IMPL_PATH = (
    Path(__file__).resolve().parent
    / "uuv_mujoco"
    / "v2.2"
    / "gui"
    / "uuv_control_gui.py"
)


def _load_impl() -> ModuleType:
    spec = importlib.util.spec_from_file_location("_uuv_control_gui_impl", _IMPL_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load GUI implementation: {_IMPL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_impl = _load_impl()
__all__ = [name for name in vars(_impl) if not name.startswith("_")]
globals().update({name: getattr(_impl, name) for name in __all__})


def main() -> int:
    return _impl.main()


if __name__ == "__main__":
    raise SystemExit(main())
