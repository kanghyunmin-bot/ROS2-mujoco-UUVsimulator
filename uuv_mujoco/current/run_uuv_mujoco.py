"""Main runtime for the UUV MuJoCo simulator.

The script keeps simulation, controls, calibration, validation, and optional
ROS2 publishing in a single entrypoint so that model tuning is reproducible.
"""

import faulthandler
import os
import signal
from pathlib import Path

import mujoco
import numpy as np

from sim.runtime.cli import build_runner_parser
from sim.runtime.control_bridge_setup import create_runtime_control_bridge_setup
from sim.runtime.env import env_flag as _env_flag
from sim.runtime.env import env_float as _env_float
from sim.runtime.parsing import to_float_array as _to_float_array
from sim.runtime.parsing import to_float_matrix as _to_float_matrix
from sim.runtime.runner_initial_setup import load_runner_initial_setup
from sim.runtime.runner_loop_setup import run_configured_simulation_loop
from sim.runtime.runner_physics_setup import create_runner_physics_setup
from sim.runtime.runner_profile_setup import load_runner_profile_setup
from sim.runtime.runner_step_setup import create_runner_step_setup

BASE_DIR = Path(__file__).resolve().parent
SCENES_DIR = BASE_DIR / "scenes"
CONFIG_DIR = BASE_DIR / "config"

MODEL_PATH = SCENES_DIR / "tank_current_scene.xml"
PROFILE_PATH = CONFIG_DIR / "sim_profiles.json"
THRUSTER_PERF_PATH = CONFIG_DIR / "thruster_performance.json"


def build_parser():
    return build_runner_parser(
        scenes_dir=SCENES_DIR,
        profile_path=PROFILE_PATH,
        thruster_perf_path=THRUSTER_PERF_PATH,
        config_dir=CONFIG_DIR,
    )


def main() -> None:
    """Parse CLI options, initialize runtime state, and execute selected mode."""
    _install_debug_traceback_signal()
    args = build_parser().parse_args()
    profile_setup = load_runner_profile_setup(
        args,
        env_get=os.environ.get,
        env_flag=_env_flag,
    )
    if profile_setup.listed_profiles:
        return

    initial_setup = load_runner_initial_setup(
        args=args,
        mujoco_module=mujoco,
        sim_profile=profile_setup.sim_profile,
        uuv_run_mode=profile_setup.uuv_run_mode,
        env_float=_env_float,
        env_flag=_env_flag,
        env_get=os.getenv,
        to_float_array=_to_float_array,
        to_float_matrix=_to_float_matrix,
    )
    control_setup = create_runtime_control_bridge_setup(
        args=args,
        model=initial_setup.model,
        data=initial_setup.data,
        initial_depth_hold=initial_setup.initial_depth_hold,
        initial_depth_runtime=initial_setup.initial_depth_runtime,
        initial_runtime_state=initial_setup.initial_runtime_state,
        water_surface_z=initial_setup.water_surface_z,
        scene_fluid_density=initial_setup.scene_fluid_density,
        base_origin_world=initial_setup.base_origin_world,
        bar30_depth_now_m=initial_setup.bar30_depth_now_m,
        world_qpos_adr=initial_setup.world_qpos_adr,
        env_flag=_env_flag,
        env_float=_env_float,
    )
    physics_setup = create_runner_physics_setup(
        args=args,
        mujoco_module=mujoco,
        np_module=np,
        config_dir=CONFIG_DIR,
        sim_profile=profile_setup.sim_profile,
        perf_cfg=profile_setup.perf_cfg,
        initial_setup=initial_setup,
        control_setup=control_setup,
        plant_replay_direct_rcout=profile_setup.plant_replay_direct_rcout,
        env_float=_env_float,
        env_flag=_env_flag,
        to_float_array=_to_float_array,
        log=lambda message: print(message, flush=True),
    )
    step_setup = create_runner_step_setup(
        args=args,
        mujoco_module=mujoco,
        initial_setup=initial_setup,
        control_setup=control_setup,
        physics_setup=physics_setup,
        plant_replay_direct_rcout=profile_setup.plant_replay_direct_rcout,
    )
    run_configured_simulation_loop(
        args=args,
        mujoco_module=mujoco,
        initial_setup=initial_setup,
        control_setup=control_setup,
        physics_setup=physics_setup,
        step_setup=step_setup,
    )


def _install_debug_traceback_signal() -> None:
    if os.environ.get("UUV_MUJOCO_FAULTHANDLER", "1").strip().lower() in {"0", "false", "no", "off"}:
        return
    try:
        faulthandler.enable(all_threads=True)
        if hasattr(signal, "SIGUSR1"):
            faulthandler.register(signal.SIGUSR1, all_threads=True, chain=False)
            print("[runtime] SIGUSR1 traceback dump enabled", flush=True)
    except Exception as exc:
        print(f"[runtime] faulthandler setup skipped: {exc}", flush=True)


if __name__ == "__main__":
    main()
