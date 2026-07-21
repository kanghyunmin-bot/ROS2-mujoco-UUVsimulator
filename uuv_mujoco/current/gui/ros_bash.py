"""Bash command assembly for GUI ROS subprocesses."""

from __future__ import annotations

import shlex
from pathlib import Path

from .config import GEOGRAPHICLIB_DATA_DIR, GEOGRAPHICLIB_GEOID_DIR, ROS_WORKSPACE_DIR
from .ros_setup_paths import existing_ros_setup_paths


def ros_bash_command(
    command: str, *, cwd: Path = ROS_WORKSPACE_DIR, include_workspace: bool = True
) -> list[str]:
    """Build a bash command that sees both the ROS distro and local rospkg install."""
    lines = ["set -e"]
    if GEOGRAPHICLIB_GEOID_DIR.is_dir():
        lines.append(f"export GEOGRAPHICLIB_DATA={shlex.quote(str(GEOGRAPHICLIB_DATA_DIR))}")
        lines.append(f"export GEOGRAPHICLIB_GEOID_PATH={shlex.quote(str(GEOGRAPHICLIB_GEOID_DIR))}")
    for setup_path in existing_ros_setup_paths(include_workspace=include_workspace):
        env_root = setup_path.parent
        if (env_root / "conda-meta").is_dir():
            lines.append(f"export CONDA_PREFIX={shlex.quote(str(env_root))}")
            lines.append(f"export PATH={shlex.quote(str(env_root / 'bin'))}:$PATH")
        lines.append(f"source {shlex.quote(str(setup_path))}")
    lines.append(f"cd {shlex.quote(str(cwd))}")
    lines.append(command)
    return ["bash", "-lc", "\n".join(lines)]


__all__ = ["ros_bash_command"]
