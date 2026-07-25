"""Runner model-IO setup with the ArduSub thruster naming contract."""

from __future__ import annotations

from pathlib import Path

from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER,
    PHYSICAL_VERTICAL_THRUSTERS,
    PHYSICAL_YAW_THRUSTERS,
)
from sim.runtime.model_io_setup import RuntimeModelIoSetup, create_runtime_model_io_setup


def create_runner_model_io_setup(
    *,
    args,
    model,
    data,
    mujoco_module,
    config_dir: Path,
    ros_bridge,
) -> RuntimeModelIoSetup:
    return create_runtime_model_io_setup(
        args=args,
        model=model,
        data=data,
        mujoco_module=mujoco_module,
        config_dir=config_dir,
        ros_bridge=ros_bridge,
        vertical_thrusters=PHYSICAL_VERTICAL_THRUSTERS,
        yaw_thrusters=PHYSICAL_YAW_THRUSTERS,
        yaw_channel_order=ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER,
    )


__all__ = ["create_runner_model_io_setup"]
