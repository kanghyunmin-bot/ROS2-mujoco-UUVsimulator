"""ROS environment and RViz config helpers for the GUI."""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
from pathlib import Path

from .config import *

def resolve_autotune_python() -> str:
    """Prefer the MuJoCo venv for offline replay/autotune helpers."""
    candidates: list[Path] = []
    env_python = os.environ.get("MJ311_PYTHON", "").strip()
    if env_python:
        candidates.append(Path(env_python).expanduser())
    env_root = os.environ.get("MJ311_ROOT", "").strip()
    if env_root:
        candidates.append(Path(env_root).expanduser() / "bin" / "python")
    candidates.append(Path(sys.executable))

    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return str(candidate)
    return sys.executable


def candidate_ros_base_setup_paths() -> list[Path]:
    """Return base ROS setup scripts, preferring the MAVROS-capable env."""
    distro = os.environ.get("ROS_DISTRO", "humble")
    # Ubuntu real-robot use:
    #   export ROS_ENV_SETUP=/opt/ros/<distro>/setup.bash
    #   source the rospkg install/setup.bash in the shell that launches the GUI
    # The conda candidates below are local macOS development fallbacks only.
    candidates = [
        os.environ.get("ROS_ENV_SETUP", ""),
        str(Path.home() / "miniconda3" / "envs" / "ros2_mavros" / "setup.bash"),
        f"/opt/ros/{distro}/setup.bash",
        str(Path.home() / "miniconda3" / "envs" / "ros2_h311" / "setup.bash"),
        str(Path.home() / "miniconda3" / "envs" / "ros2" / "setup.bash"),
        os.environ.get("CONDA_PREFIX", "") + "/setup.bash" if os.environ.get("CONDA_PREFIX") else "",
    ]
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
        for candidate in [
            ROS_WORKSPACE_DIR / "install" / "setup.bash",
            APP_ROOT / "install" / "setup.bash",
        ]:
            if candidate.is_file() and candidate not in paths:
                paths.append(candidate)
    return paths


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


def prepare_ros2_rviz_config(source_path: Path = ROS_PACKAGE_RVIZ_CONFIG) -> Path:
    """Create a ROS2-compatible RViz copy without modifying the rospkg source."""
    replacements = {
        "Class: rviz/Displays": "Class: rviz_common/Displays",
        "Class: rviz/Selection": "Class: rviz_common/Selection",
        "Class: rviz/Tool Properties": "Class: rviz_common/Tool Properties",
        "Class: rviz/Views": "Class: rviz_common/Views",
        "Class: rviz/Time": "Class: rviz_common/Time",
        "Class: rviz/Grid": "Class: rviz_default_plugins/Grid",
        "Class: rviz/TF": "Class: rviz_default_plugins/TF",
        "Class: rviz/RobotModel": "Class: rviz_default_plugins/RobotModel",
        "Class: rviz/Odometry": "Class: rviz_default_plugins/Odometry",
        "Class: rviz/Marker": "Class: rviz_default_plugins/Marker",
        "Class: rviz/Interact": "Class: rviz_default_plugins/Interact",
        "Class: rviz/MoveCamera": "Class: rviz_default_plugins/MoveCamera",
        "Class: rviz/Select": "Class: rviz_default_plugins/Select",
        "Class: rviz/FocusCamera": "Class: rviz_default_plugins/FocusCamera",
        "Class: rviz/Measure": "Class: rviz_default_plugins/Measure",
        "Class: rviz/SetInitialPose": "Class: rviz_default_plugins/SetInitialPose",
        "Class: rviz/SetGoal": "Class: rviz_default_plugins/SetGoal",
        "Class: rviz/PublishPoint": "Class: rviz_default_plugins/PublishPoint",
        "Class: rviz/Orbit": "Class: rviz_default_plugins/Orbit",
    }
    text = source_path.read_text(encoding="utf-8", errors="replace")
    for old, new in replacements.items():
        text = text.replace(old, new)
    if "Value: /sim/odom" not in text:
        sim_odom_display = """    - Alpha: 1
      Class: rviz_default_plugins/Odometry
      Color: 25; 170; 255
      Enabled: true
      Keep: 200
      Length: 1.4
      Name: Sim Odom
      Position Use Topic: true
      Topic:
        Value: /sim/odom
      Value: true
"""
        text = text.replace("  Enabled: true\n  Global Options:", f"{sim_odom_display}  Enabled: true\n  Global Options:")
        if "- /Sim Odom1" not in text:
            text = text.replace("        - /Odometry1\n", "        - /Odometry1\n        - /Sim Odom1\n")
    ROS2_RVIZ_COMPAT_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    if not ROS2_RVIZ_COMPAT_CONFIG.exists() or ROS2_RVIZ_COMPAT_CONFIG.read_text(
        encoding="utf-8", errors="replace"
    ) != text:
        ROS2_RVIZ_COMPAT_CONFIG.write_text(text, encoding="utf-8")
    return ROS2_RVIZ_COMPAT_CONFIG


def prepare_ping360_rviz_config() -> Path:
    """Create a Ping360 RViz view outside the real-robot rospkg."""
    text = """Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Ping360 Image1
      Splitter Ratio: 0.5
    Tree Height: 218
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 255
      Median window: 5
      Min Value: 0
      Name: Ping360 Image
      Normalize Range: false
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /ping360/scan_image
      Value: true
  Enabled: true
  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.6
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.8
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 720
  Hide Left Dock: false
  Hide Right Dock: true
  Time:
    collapsed: false
  Width: 960
  X: 120
  Y: 120
"""
    PING360_RVIZ_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    if not PING360_RVIZ_CONFIG.exists() or PING360_RVIZ_CONFIG.read_text(
        encoding="utf-8", errors="replace"
    ) != text:
        PING360_RVIZ_CONFIG.write_text(text, encoding="utf-8")
    return PING360_RVIZ_CONFIG
