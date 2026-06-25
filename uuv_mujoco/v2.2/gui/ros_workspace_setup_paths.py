"""Workspace ROS setup-script discovery for GUI launch helpers."""

from __future__ import annotations

from pathlib import Path

from .config import APP_ROOT, ROS_WORKSPACE_DIR


def workspace_setup_candidates() -> list[Path]:
    return [
        ROS_WORKSPACE_DIR / "install" / "setup.bash",
        APP_ROOT / "install" / "setup.bash",
    ]


def append_existing_workspace_setup_paths(paths: list[Path]) -> None:
    for candidate in workspace_setup_candidates():
        if candidate.is_file() and candidate not in paths:
            paths.append(candidate)


__all__ = ["append_existing_workspace_setup_paths", "workspace_setup_candidates"]
