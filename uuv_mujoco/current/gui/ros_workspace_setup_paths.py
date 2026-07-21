"""Workspace ROS setup-script discovery for GUI launch helpers."""

from __future__ import annotations

from pathlib import Path

from .config import ROS_WORKSPACE_DIR


def workspace_setup_candidates() -> list[Path]:
    # The canonical deployable workspace is rospkg.  Do not source the legacy
    # workspace-root install here: when the GUI itself already inherited the
    # rospkg overlay, adding root/local_setup prepends stale duplicate packages
    # and a later rospkg/setup source is skipped by colcon's duplicate-prefix
    # guard.  That made GUI Start Pinger execute the archived audio estimator.
    return [
        ROS_WORKSPACE_DIR / "install" / "setup.bash",
    ]


def append_existing_workspace_setup_paths(paths: list[Path]) -> None:
    for candidate in workspace_setup_candidates():
        if candidate.is_file() and candidate not in paths:
            paths.append(candidate)


__all__ = ["append_existing_workspace_setup_paths", "workspace_setup_candidates"]
