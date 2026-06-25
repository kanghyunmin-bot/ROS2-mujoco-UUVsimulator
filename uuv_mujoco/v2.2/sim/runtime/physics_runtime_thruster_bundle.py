"""Create and bind thruster parameter and SITL servo runtimes."""

from __future__ import annotations

import os
from typing import Any

from sim.runtime.physics_runtime_thruster_types import ThrusterParamServoRuntime
from sim.runtime.sitl_servo_runtime import create_and_bind_sitl_servo_runtime
from sim.runtime.thruster_param_runtime import ThrusterParameterRuntime


def create_thruster_param_and_servo_runtimes(
    *,
    args: Any,
    all_thruster_names: list[str],
    servo_map: list[int],
    servo_signs: list[float],
    plant_replay_direct_rcout: bool,
    ros_bridge_runtime: Any,
    thruster_params_path: Any,
    sim_profile: dict[str, Any],
    vertical_thrusters: list[str],
    horizontal_thrusters: list[str],
    perf_cfg: dict[str, Any],
    log,
) -> ThrusterParamServoRuntime:
    thruster_param_runtime = ThrusterParameterRuntime.create(all_thruster_names)
    sitl_servo_runtime = create_and_bind_sitl_servo_runtime(
        all_thruster_names=all_thruster_names,
        raw_map=servo_map,
        servo_signs=servo_signs,
        sitl_servo_scale=float(args.sitl_servo_scale),
        sitl_enabled=bool(args.sitl),
        plant_replay_direct_rcout=plant_replay_direct_rcout,
        ros_bridge_runtime=ros_bridge_runtime,
        log=log,
    )

    thruster_param_runtime.load(
        path=thruster_params_path,
        thruster_names=all_thruster_names,
        sim_profile=sim_profile,
        vertical_thrusters=vertical_thrusters,
        horizontal_thrusters=horizontal_thrusters,
        env_get=os.getenv,
        log=log,
    )
    thruster_param_runtime.log_summary(
        perf_cfg=perf_cfg,
        all_thruster_names=all_thruster_names,
        yaw_thrusters=horizontal_thrusters,
        log=log,
    )
    return ThrusterParamServoRuntime(
        thruster_param_runtime=thruster_param_runtime,
        sitl_servo_runtime=sitl_servo_runtime,
    )


__all__ = ["create_thruster_param_and_servo_runtimes"]
