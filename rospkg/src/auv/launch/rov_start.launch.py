"""Expose the organization launch name while retaining the legacy package."""

from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    legacy = (
        Path(get_package_share_directory("hit25_auv_ros2"))
        / "launch"
        / "rov_start.launch.py"
    )
    return runpy.run_path(str(legacy))["generate_launch_description"]()
