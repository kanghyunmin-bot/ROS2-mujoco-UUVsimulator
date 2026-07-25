"""ROS setup-script probe helpers."""

from __future__ import annotations

import shlex
import subprocess
from pathlib import Path


def setup_path_has_ros_package(setup_path: Path, package_name: str) -> bool:
    probe = (
        f"source {shlex.quote(str(setup_path))} >/dev/null 2>&1 && "
        f"ros2 pkg prefix {shlex.quote(package_name)} >/dev/null 2>&1"
    )
    try:
        result = subprocess.run(
            ["bash", "-lc", probe],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
            check=False,
        )
    except Exception:
        return False
    return result.returncode == 0


__all__ = ["setup_path_has_ros_package"]
