"""Profile, run-mode, and thruster-performance setup for the runner."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from sim.physics.fluid_model import (
    normalize_fluid_model,
    validate_requested_fluid_model_profile,
)
from sim.physics.profile_runtime import select_runtime_profile
from sim.physics.thruster_performance import select_thruster_performance_config
from sim.runtime.run_mode import resolve_runtime_mode


@dataclass(frozen=True)
class RunnerProfileSetup:
    listed_profiles: bool
    runtime_mode: Any
    uuv_run_mode: str
    plant_replay_direct_rcout: bool
    sim_profile: dict
    active_thruster_voltage: float
    perf_cfg: Any


def load_runner_profile_setup(
    args,
    *,
    env_get: Callable[[str, Any], Any],
    env_flag: Callable[[str, bool], bool],
) -> RunnerProfileSetup:
    runtime_mode = resolve_runtime_mode(env_get, env_flag)
    requested_fluid_model = str(args.fluid_model)
    args.fluid_model = normalize_fluid_model(requested_fluid_model)

    profile_selection = select_runtime_profile(args)
    if profile_selection.listed_profiles:
        return RunnerProfileSetup(
            listed_profiles=True,
            runtime_mode=runtime_mode,
            uuv_run_mode=runtime_mode.name,
            plant_replay_direct_rcout=bool(runtime_mode.plant_replay_direct_rcout),
            sim_profile={},
            active_thruster_voltage=0.0,
            perf_cfg=None,
        )

    try:
        validate_requested_fluid_model_profile(
            requested_fluid_model,
            profile_selection.sim_profile,
        )
    except ValueError as exc:
        print(f"[physics] {exc}", flush=True)
        raise SystemExit(2) from exc

    perf_cfg = select_thruster_performance_config(
        args=args,
        active_thruster_voltage=float(profile_selection.active_thruster_voltage),
        path=Path(args.thruster_perf_file),
        plant_replay_direct_rcout=bool(runtime_mode.plant_replay_direct_rcout),
    )
    return RunnerProfileSetup(
        listed_profiles=False,
        runtime_mode=runtime_mode,
        uuv_run_mode=runtime_mode.name,
        plant_replay_direct_rcout=bool(runtime_mode.plant_replay_direct_rcout),
        sim_profile=profile_selection.sim_profile,
        active_thruster_voltage=float(profile_selection.active_thruster_voltage),
        perf_cfg=perf_cfg,
    )


__all__ = ["RunnerProfileSetup", "load_runner_profile_setup"]
