"""Fluid contract, model IO, and physics setup for the runner."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from sim.runtime.model_io_setup import RuntimeModelIoSetup
from sim.runtime.runner_control_path_log import log_runtime_control_path
from sim.runtime.runner_fluid_contract_setup import create_runner_fluid_contract_setup
from sim.runtime.runner_model_io_contract import create_runner_model_io_setup
from sim.runtime.runner_runtime_physics_contract import create_runner_runtime_physics


@dataclass
class RunnerPhysicsSetup:
    model_io: RuntimeModelIoSetup
    physics: Any
    fluid_model: str
    use_custom_hydrodynamics: bool


def create_runner_physics_setup(
    *,
    args,
    mujoco_module,
    np_module,
    config_dir: Path,
    sim_profile: dict,
    perf_cfg: Any,
    initial_setup,
    control_setup,
    plant_replay_direct_rcout: bool,
    env_float,
    env_flag,
    to_float_array,
    log: Callable[[str], None],
) -> RunnerPhysicsSetup:
    fluid_setup = create_runner_fluid_contract_setup(args=args, initial_setup=initial_setup)
    log_runtime_control_path(args)
    model_io = create_runner_model_io_setup(
        args=args,
        model=initial_setup.model,
        data=initial_setup.data,
        mujoco_module=mujoco_module,
        config_dir=config_dir,
        ros_bridge=control_setup.ros_bridge_runtime.get(),
    )
    physics = create_runner_runtime_physics(
        args=args,
        mujoco_module=mujoco_module,
        np_module=np_module,
        sim_profile=sim_profile,
        perf_cfg=perf_cfg,
        initial_setup=initial_setup,
        control_setup=control_setup,
        model_io=model_io,
        use_custom_hydrodynamics=fluid_setup.use_custom_hydrodynamics,
        plant_replay_direct_rcout=plant_replay_direct_rcout,
        env_float=env_float,
        env_flag=env_flag,
        to_float_array=to_float_array,
        log=log,
    )
    return RunnerPhysicsSetup(
        model_io=model_io,
        physics=physics,
        fluid_model=fluid_setup.fluid_model,
        use_custom_hydrodynamics=fluid_setup.use_custom_hydrodynamics,
    )


__all__ = ["RunnerPhysicsSetup", "create_runner_physics_setup", "log_runtime_control_path"]
