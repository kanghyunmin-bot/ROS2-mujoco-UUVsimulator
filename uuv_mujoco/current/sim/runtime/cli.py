"""Command-line interface contract for the MuJoCo UUV runtime."""

from __future__ import annotations

import argparse
from pathlib import Path

from .cli_initial_state import add_initial_state_args
from .cli_profile import add_profile_args
from .cli_ros2 import add_ros2_args
from .cli_sensor_video import add_ping360_args, add_qgc_video_args
from .cli_sitl import add_sitl_args
from .cli_viewer import add_viewer_args


def build_runner_parser(
    *,
    scenes_dir: Path,
    profile_path: Path,
    thruster_perf_path: Path,
    config_dir: Path,
) -> argparse.ArgumentParser:
    """Build the CLI parser for the UUV MuJoCo runtime entrypoint."""

    parser = argparse.ArgumentParser(description="UUV MuJoCo runner")
    add_profile_args(
        parser,
        scenes_dir=scenes_dir,
        profile_path=profile_path,
        thruster_perf_path=thruster_perf_path,
    )
    add_ros2_args(parser)
    add_ping360_args(parser, config_dir=config_dir)
    add_qgc_video_args(parser)
    add_sitl_args(parser)
    add_initial_state_args(parser)
    add_viewer_args(parser)
    return parser


__all__ = ["build_runner_parser"]
