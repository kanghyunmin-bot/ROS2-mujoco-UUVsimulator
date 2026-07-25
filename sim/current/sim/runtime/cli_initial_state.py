"""Initial-state CLI options for the UUV MuJoCo runner."""

from __future__ import annotations

import argparse


def add_initial_state_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--initial-depth-m",
        type=float,
        default=None,
        help=(
            "Set initial base_link depth relative to the water surface before runtime starts. "
            "Positive is underwater; negative starts above the water and lets gravity drop the vehicle."
        ),
    )
    parser.add_argument(
        "--initial-bar30-depth-m",
        type=str,
        default=None,
        help=(
            "Set the initial pose so the Bar30 sensor site is at this positive-down "
            "depth relative to the water surface before sensors are published. "
            "Use 'auto' to compute the minimum fully-submerged start depth from "
            "thruster sites, MuJoCo ellipsoid fluid geoms, and buoyancy proxies. "
            "This takes precedence over --initial-depth-m for SITL depth-hold starts."
        ),
    )
    parser.add_argument(
        "--initial-position-xy",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        default=None,
        help=(
            "Set the initial base_link horizontal position in world/ENU metres. "
            "Real RCOU plant replays use this to match /mavros/local_position/pose x/y."
        ),
    )
    parser.add_argument(
        "--initial-rpy-rad",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=None,
        help=(
            "Set the initial base_link attitude as ROS/ENU roll, pitch, yaw in radians. "
            "Useful for rosbag replays that start mid-run instead of from a level vehicle."
        ),
    )
    parser.add_argument(
        "--initial-depth-hold-target-m",
        type=float,
        default=None,
        help=(
            "Deprecated compatibility value. Runtime depth target switching is disabled; "
            "use --initial-depth-m to set the starting pose before sensors are published."
        ),
    )
    parser.add_argument(
        "--hold-initial-depth-until-release",
        action="store_true",
        help="Pin the vehicle at the initial/target depth until /mujoco/release_initial_depth_hold is called.",
    )
    parser.add_argument(
        "--release-linear-velocity-body",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "VZ"),
        default=None,
        help=(
            "Body-frame linear velocity [m/s] applied when the artificial "
            "initial-depth hold is released. Use this for rosbag replays that "
            "start while the real vehicle is already moving."
        ),
    )
    parser.add_argument(
        "--release-angular-velocity-body",
        type=float,
        nargs=3,
        metavar=("WX", "WY", "WZ"),
        default=None,
        help=(
            "Body-frame angular velocity [rad/s] applied when the artificial "
            "initial-depth hold is released. Use this for rosbag replays that "
            "start while the real vehicle is already rotating."
        ),
    )
