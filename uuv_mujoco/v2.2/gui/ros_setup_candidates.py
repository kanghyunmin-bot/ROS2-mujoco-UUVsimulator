"""ROS setup-script candidate discovery."""

from __future__ import annotations

import os
from pathlib import Path


def raw_ros_base_setup_candidates() -> list[str]:
    distro = os.environ.get("ROS_DISTRO", "humble")
    return [
        os.environ.get("ROS_ENV_SETUP", ""),
        str(Path.home() / "miniconda3" / "envs" / "ros2_mavros" / "setup.bash"),
        f"/opt/ros/{distro}/setup.bash",
        str(Path.home() / "miniconda3" / "envs" / "ros2_h311" / "setup.bash"),
        str(Path.home() / "miniconda3" / "envs" / "ros2" / "setup.bash"),
        os.environ.get("CONDA_PREFIX", "") + "/setup.bash" if os.environ.get("CONDA_PREFIX") else "",
    ]


def existing_unique_setup_paths(candidates: list[str]) -> list[Path]:
    paths: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).expanduser()
        key = str(path)
        if key in seen or not path.is_file():
            continue
        seen.add(key)
        paths.append(path)
    return paths


__all__ = ["existing_unique_setup_paths", "raw_ros_base_setup_candidates"]
