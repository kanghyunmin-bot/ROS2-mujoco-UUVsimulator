"""Synthetic sonar and video CLI options for the UUV MuJoCo runner."""

from __future__ import annotations

import argparse
from pathlib import Path


def add_ping360_args(parser: argparse.ArgumentParser, *, config_dir: Path) -> None:
    parser.add_argument(
        "--no-ping360",
        action="store_true",
        help="Disable the synthetic Ping360 sonar ROS2 topics.",
    )
    parser.add_argument(
        "--ping360-config",
        type=str,
        default=str(config_dir / "ping360.json"),
        help="Ping360 JSON configuration path.",
    )
    parser.add_argument(
        "--ping360-range-m",
        type=float,
        default=None,
        help="Override requested Ping360 range in meters; firmware model clamps to 0.75-50.0 m.",
    )
    parser.add_argument(
        "--ping360-num-steps",
        type=int,
        default=None,
        help="Override Ping360 motor steps per ping, 1-10 gradians.",
    )
    parser.add_argument(
        "--ping360-interface",
        choices=("usb", "ethernet", "rs485"),
        default=None,
        help="Override Ping360 communications interface for scan-time estimation.",
    )
    parser.add_argument(
        "--ping360-gain",
        type=int,
        default=None,
        help="Override Ping360 gain setting, 0=low, 1=normal, 2=high.",
    )


def add_qgc_video_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--qgc-video",
        action="store_true",
        help="Stream stereo_left camera directly to QGroundControl over UDP H264",
    )
    parser.add_argument(
        "--qgc-video-host",
        type=str,
        default="127.0.0.1",
        help="QGroundControl video UDP target host",
    )
    parser.add_argument(
        "--qgc-video-port",
        type=int,
        default=5600,
        help="QGroundControl video UDP target port",
    )
    parser.add_argument(
        "--qgc-video-fps",
        type=float,
        default=15.0,
        help="QGC video output FPS",
    )
    parser.add_argument(
        "--qgc-video-width",
        type=int,
        default=640,
        help="QGC video output width",
    )
    parser.add_argument(
        "--qgc-video-height",
        type=int,
        default=360,
        help="QGC video output height",
    )
    parser.add_argument(
        "--qgc-video-bitrate-kbps",
        type=int,
        default=2600,
        help="QGC video H264 bitrate in kbps",
    )
