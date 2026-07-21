"""Resolve the active MuJoCo runtime alias used by freshness checks."""

from __future__ import annotations

import os
from pathlib import Path


def alias_text(active_alias: Path) -> str:
    if active_alias.is_symlink():
        try:
            return os.readlink(active_alias)
        except OSError:
            return "<unreadable-symlink>"
    if active_alias.exists():
        return str(active_alias.resolve())
    return "<missing>"


def resolve_runtime_dir(workspace: Path, runtime_dir: Path | None) -> Path:
    active_alias = workspace / "uuv_mujoco" / "current"
    if runtime_dir is not None:
        resolved = runtime_dir
    elif os.environ.get("UUV_MUJOCO_RUNTIME_DIR"):
        resolved = Path(os.environ["UUV_MUJOCO_RUNTIME_DIR"])
    else:
        resolved = active_alias
    if resolved.is_absolute():
        return Path(os.path.abspath(os.fspath(resolved)))
    return Path(os.path.abspath(os.fspath(workspace / resolved)))


def same_path_text(left: Path, right: Path) -> bool:
    return os.path.abspath(os.fspath(left)) == os.path.abspath(os.fspath(right))
