"""Environment contract builder for GUI-started MuJoCo/SITL runs."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

from .sim_stack_env_defaults import (
    apply_command_endpoint_defaults,
    apply_mujoco_viewer_display_defaults,
    apply_native_stable_defaults,
    profile_defaults,
)
from .sim_stack_env_forced import apply_forced_gui_contract
from .sim_stack_env_modes import normalize_ekf_contract, normalize_run_mode


def build_gui_sim_stack_env(
    base_env: Mapping[str, str],
    *,
    backend: str,
    sim_stack_dir: Path,
) -> dict[str, str]:
    """Build the environment contract used by GUI-started simulator runs."""

    explicit_keys = set(base_env)
    env = dict(base_env)
    env["PYTHONUNBUFFERED"] = "1"

    run_mode = normalize_run_mode(env.get("UUV_RUN_MODE", "closed_loop"))
    env["UUV_RUN_MODE"] = run_mode

    ekf_contract = normalize_ekf_contract(env.get("UUV_EKF_CONTRACT", "althold_baro"))
    env["UUV_EKF_CONTRACT"] = ekf_contract

    apply_forced_gui_contract(
        env,
        run_mode=run_mode,
        ekf_contract=ekf_contract,
        sim_stack_dir=sim_stack_dir,
    )
    apply_command_endpoint_defaults(env, backend=backend)
    apply_mujoco_viewer_display_defaults(env, explicit_keys=explicit_keys)
    if str(backend).strip().lower() != "docker":
        apply_native_stable_defaults(env, sim_stack_dir=sim_stack_dir, explicit_keys=explicit_keys)
    return env


__all__ = ["build_gui_sim_stack_env", "normalize_run_mode", "normalize_ekf_contract", "profile_defaults"]
