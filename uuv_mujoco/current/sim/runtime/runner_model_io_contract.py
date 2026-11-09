"""Runner model-IO setup with the ArduSub thruster naming contract."""

from __future__ import annotations

from pathlib import Path

from sim.runtime.vehicle_manifest import runtime_thruster_mapping
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
    horizontal, vertical, channel_order, _ = runtime_thruster_mapping(args)
    return create_runtime_model_io_setup(
        args=args,
        model=model,
        data=data,
        mujoco_module=mujoco_module,
        config_dir=config_dir,
        ros_bridge=ros_bridge,
        vertical_thrusters=vertical,
        yaw_thrusters=horizontal,
        yaw_channel_order=tuple(name for name in channel_order if name in horizontal),
    )


__all__ = ["create_runner_model_io_setup"]
