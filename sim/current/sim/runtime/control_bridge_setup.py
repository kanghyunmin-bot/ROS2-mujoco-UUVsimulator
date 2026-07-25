"""Runtime control and ROS bridge setup for the main MuJoCo runner."""

from __future__ import annotations

import threading
from typing import Callable

import numpy as np

from .control_bridge_command_setup import (
    create_runtime_command_state,
    resolve_sitl_direct_command_policy,
    update_runtime_command_from_ros,
)
from .control_bridge_initial_depth_services import install_runtime_initial_depth_services
from .control_bridge_real_start_status import create_runtime_real_start_status
from .control_bridge_setup_types import RuntimeControlBridgeSetup
from .ros_bridge_launcher import create_ros_bridge_or_exit
from .ros_bridge_runtime import RosBridgeRuntime
from .viewer_controls import ViewerControlState


def create_runtime_control_bridge_setup(
    *,
    args,
    model,
    data,
    initial_depth_hold: dict,
    initial_depth_runtime,
    initial_runtime_state,
    water_surface_z: float,
    base_origin_world: Callable[[], np.ndarray],
    bar30_depth_now_m: Callable[[], float],
    world_qpos_adr: int,
    env_flag: Callable[[str, bool], bool],
    env_float: Callable[[str, float], float],
) -> RuntimeControlBridgeSetup:
    """Create direct-command state, ROS bridge, initial-depth service, and real-start status."""

    command_state = create_runtime_command_state()
    sitl_allow_direct_cmd = resolve_sitl_direct_command_policy(args=args, env_flag=env_flag)
    stop_event = threading.Event()
    viewer_controls = ViewerControlState.create(args=args, env_float=env_float)

    def apply_ros_cmd(forward: float, sway: float, yaw: float, heave: float) -> None:
        update_runtime_command_from_ros(
            command_state,
            args=args,
            sitl_allow_direct_cmd=sitl_allow_direct_cmd,
            forward=forward,
            sway=sway,
            yaw=yaw,
            heave=heave,
        )

    ros_bridge_runtime = RosBridgeRuntime(
        create_ros_bridge_or_exit(
            args=args,
            model=model,
            command_callback=apply_ros_cmd,
            cmd_limit=command_state.max_value,
        )
    )

    (
        release_initial_depth_hold,
        process_pending_initial_depth_release,
        initial_depth_services,
    ) = install_runtime_initial_depth_services(
        ros_bridge_runtime=ros_bridge_runtime,
        initial_depth_hold=initial_depth_hold,
        initial_depth_runtime=initial_depth_runtime,
    )
    real_start_status = create_runtime_real_start_status(
        ros_bridge_runtime=ros_bridge_runtime,
        env_float=env_float,
        initial_runtime_state=initial_runtime_state,
        initial_depth_hold=initial_depth_hold,
        water_surface_z=water_surface_z,
        base_origin_world=base_origin_world,
        bar30_depth_now_m=bar30_depth_now_m,
        data=data,
        model=model,
        world_qpos_adr=world_qpos_adr,
    )

    return RuntimeControlBridgeSetup(
        command_state=command_state,
        sitl_allow_direct_cmd=bool(sitl_allow_direct_cmd),
        stop_event=stop_event,
        viewer_controls=viewer_controls,
        ros_bridge_runtime=ros_bridge_runtime,
        release_initial_depth_hold=release_initial_depth_hold,
        process_pending_initial_depth_release=process_pending_initial_depth_release,
        real_start_status=real_start_status,
        initial_depth_services=initial_depth_services,
    )


__all__ = ["RuntimeControlBridgeSetup", "create_runtime_control_bridge_setup"]
