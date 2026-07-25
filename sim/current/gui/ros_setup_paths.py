"""ROS setup-script discovery for GUI launch helpers."""

from __future__ import annotations

import os
from pathlib import Path

from .ros_setup_candidates import existing_unique_setup_paths, raw_ros_base_setup_candidates
from .ros_setup_probe import setup_path_has_ros_package
from .ros_workspace_setup_paths import append_existing_workspace_setup_paths


def candidate_ros_base_setup_paths() -> list[Path]:
    """Return base ROS setup scripts, preferring the MAVROS-capable env."""
    # Ubuntu real-robot use:
    #   export ROS_ENV_SETUP=/opt/ros/<distro>/setup.bash
    #   source the rospkg install/setup.bash in the shell that launches the GUI
    # The conda candidates below are local macOS development fallbacks only.
    return existing_unique_setup_paths(raw_ros_base_setup_candidates())


def selected_ros_base_setup_path() -> Path | None:
    candidates = candidate_ros_base_setup_paths()
    explicit = os.environ.get("ROS_ENV_SETUP")
    if explicit:
        explicit_path = Path(explicit).expanduser()
        if explicit_path.is_file():
            return explicit_path
    for candidate in candidates:
        if setup_path_has_ros_package(candidate, "mavros"):
            return candidate
    return candidates[0] if candidates else None


def existing_ros_setup_paths(*, include_workspace: bool = True) -> list[Path]:
    """Return setup scripts without mixing multiple base ROS prefixes."""
    paths: list[Path] = []
    base_setup = selected_ros_base_setup_path()
    if base_setup is not None:
        paths.append(base_setup)
    if include_workspace:
        append_existing_workspace_setup_paths(paths)
    return paths


__all__ = [
    "candidate_ros_base_setup_paths",
    "existing_ros_setup_paths",
    "selected_ros_base_setup_path",
    "setup_path_has_ros_package",
]
