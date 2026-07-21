"""Typed container for runtime command and ROS bridge setup."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable

from .command_state import RuntimeCommandState
from .real_start_runtime import RealStartRuntimeStatus
from .ros_bridge_runtime import RosBridgeRuntime
from .viewer_controls import ViewerControlState


@dataclass
class RuntimeControlBridgeSetup:
    command_state: RuntimeCommandState
    sitl_allow_direct_cmd: bool
    stop_event: threading.Event
    viewer_controls: ViewerControlState
    ros_bridge_runtime: RosBridgeRuntime
    release_initial_depth_hold: Callable[[str], bool]
    process_pending_initial_depth_release: Callable[[], bool]
    real_start_status: RealStartRuntimeStatus
    initial_depth_services: Any


__all__ = ["RuntimeControlBridgeSetup"]
