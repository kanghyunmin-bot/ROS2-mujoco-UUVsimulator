"""ArduPilot SITL CLI options for the UUV MuJoCo runner."""

from __future__ import annotations

import argparse

from .cli_common import env_float


def add_sitl_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Enable ArduPilot SITL JSON bridge (sends IMU/Pose)",
    )
    parser.add_argument(
        "--sitl-ip",
        type=str,
        default="127.0.0.1",
        help="ArduPilot SITL JSON interface IP",
    )
    parser.add_argument(
        "--sitl-port",
        type=int,
        default=9002,
        help="ArduPilot SITL JSON servo recv/listen Port (ArduPilot sends servo packets here)",
    )
    parser.add_argument(
        "--sitl-send-port",
        type=int,
        default=9003,
        help="ArduPilot SITL JSON sensor send Port (ArduPilot expects JSON sensor packets here)",
    )
    parser.add_argument(
        "--sitl-mavlink-endpoint",
        type=str,
        default="udpin:0.0.0.0:14660",
        help="MAVLink endpoint to receive SERVO_OUTPUT_RAW for SITL thruster commands. Use 'none' to disable MAVLink servo input and rely on JSON UDP control only.",
    )
    parser.add_argument(
        "--sitl-mavlink-servo-hz",
        type=float,
        default=env_float("UUV_SITL_MAVLINK_SERVO_HZ", 25.0),
        help="Requested SERVO_OUTPUT_RAW rate over MAVLink (Hz).",
    )
    parser.add_argument(
        "--sitl-mavlink-target-sysid",
        type=int,
        default=0,
        help="Target vehicle sysid expected for SERVO_OUTPUT_RAW (0=auto from heartbeat).",
    )
    parser.add_argument(
        "--sitl-mavlink-target-compid",
        type=int,
        default=0,
        help="Target vehicle compid expected for SERVO_OUTPUT_RAW (0=auto from heartbeat).",
    )
    parser.add_argument(
        "--sitl-mavlink-source-sysid",
        type=int,
        default=254,
        help="Source sysid for MuJoCo MAVLink control messages. ArduSub accepts pilot input only from SYSID_MYGCS.",
    )
    parser.add_argument(
        "--sitl-mavlink-source-compid",
        type=int,
        default=240,
        help="Source component id for MuJoCo MAVLink listener.",
    )
    parser.add_argument(
        "--sitl-servo-scale",
        type=float,
        default=1.0,
        help=(
            "Scale applied to direct-thruster normalized command from SITL PWM "
            "(default: 1.0)."
        ),
    )
    parser.add_argument(
        "--thruster-loop-hz",
        type=float,
        default=env_float("UUV_THRUSTER_LOOP_HZ", 80.0),
        help="Thruster force update rate (Hz), decoupled from physics timestep.",
    )
