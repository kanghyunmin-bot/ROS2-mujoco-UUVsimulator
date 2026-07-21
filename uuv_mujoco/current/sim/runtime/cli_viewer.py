"""Viewer CLI options for the UUV MuJoCo runner."""

from __future__ import annotations

import argparse
import os

from .cli_common import env_float


def add_viewer_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run real-time simulation loop without GLFW viewer",
    )
    parser.add_argument(
        "--viewer-fps",
        type=float,
        default=env_float("UUV_MUJOCO_VIEWER_FPS", 30.0),
        help="Maximum passive MuJoCo viewer refresh rate in Hz.",
    )
    parser.add_argument(
        "--viewer-debug",
        action="store_true",
        help="Draw heavy viewer debug overlays such as thruster arrows, bubbles, labels, and sensor markers",
    )
    parser.add_argument(
        "--enable-viewer-pause",
        action="store_true",
        help="Allow the MuJoCo viewer spacebar/pause state to stop the simulation loop",
    )
    parser.add_argument(
        "--viewer-camera-mode",
        type=str,
        default=os.environ.get("UUV_MUJOCO_VIEWER_CAMERA_MODE", ""),
        choices=("", "free", "follow", "course_overview", "course_side", "stereo_left", "stereo_right"),
        help=(
            "Initial MuJoCo viewer camera. Empty defaults to follow in SITL and free otherwise. "
            "Use course_overview/course_side for field cameras or stereo_left/stereo_right for fixed onboard robot cameras."
        ),
    )
