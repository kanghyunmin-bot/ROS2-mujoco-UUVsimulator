"""Direct-command state setup for the MuJoCo runner."""

from __future__ import annotations

import os
import time

import numpy as np

from .command_state import RuntimeCommandState


def create_runtime_command_state() -> RuntimeCommandState:
    return RuntimeCommandState(
        timeout_s=float(
            np.clip(
                float(os.getenv("ROS2_UUV_CMD_TIMEOUT_S", "0.45")),
                0.1,
                2.0,
            )
        )
    )


def resolve_sitl_direct_command_policy(*, args, env_flag) -> bool:
    sitl_allow_direct_cmd = env_flag("ROS2_UUV_SITL_ALLOW_DIRECT_CMD", False)
    if args.sitl and not sitl_allow_direct_cmd:
        print(
            "[control] SITL closed-loop authority: direct MuJoCo command fallback is disabled. "
            "Set ROS2_UUV_SITL_ALLOW_DIRECT_CMD=1 only for smoke/debug.",
            flush=True,
        )
    return bool(sitl_allow_direct_cmd)


def update_runtime_command_from_ros(
    command_state: RuntimeCommandState,
    *,
    args,
    sitl_allow_direct_cmd: bool,
    forward: float,
    sway: float,
    yaw: float,
    heave: float,
) -> None:
    command_state.update(
        forward=forward,
        sway=sway,
        yaw=yaw,
        heave=heave,
        now_wall=time.monotonic(),
        allow=not (args.sitl and not sitl_allow_direct_cmd),
    )


__all__ = [
    "create_runtime_command_state",
    "resolve_sitl_direct_command_policy",
    "update_runtime_command_from_ros",
]
