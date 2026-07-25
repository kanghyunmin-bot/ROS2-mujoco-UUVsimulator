"""Runtime control-path startup log messages."""

from __future__ import annotations


def log_runtime_control_path(args) -> None:
    if args.sitl:
        print("[runtime] Local manual control path removed in SITL mode (QGC remote control only).", flush=True)
    else:
        print("[runtime] Manual keyboard/joystick path removed. Use ROS2 /cmd_vel or SITL/QGC control.", flush=True)
    if args.sitl and args.ros2:
        print(
            "[runtime] SITL RC override/manual-control topics are forwarded to "
            "ArduSub; direct MuJoCo override is disabled unless explicitly enabled.",
            flush=True,
        )


__all__ = ["log_runtime_control_path"]
