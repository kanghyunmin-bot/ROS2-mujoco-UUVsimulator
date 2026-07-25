"""Runtime wrapper for thruster tuning parameters and diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from sim.runtime.thruster_param_runtime_loader import load_runtime_thruster_parameters
from sim.runtime.thruster_param_runtime_state import create_thruster_parameter_state
from sim.runtime.thruster_param_runtime_summary import log_thruster_runtime_summary


@dataclass
class ThrusterParameterRuntime:
    global_params: dict[str, Any]
    scale: dict[str, float]
    direct_scale: dict[str, float]
    reverse_asymmetry: dict[str, float | None]
    tau_up: dict[str, float | None]
    tau_down: dict[str, float | None]

    @classmethod
    def create(cls, thruster_names: list[str]) -> "ThrusterParameterRuntime":
        return cls(**create_thruster_parameter_state(thruster_names))

    def load(
        self,
        *,
        path: Path,
        thruster_names: list[str],
        sim_profile: dict,
        vertical_thrusters: list[str],
        horizontal_thrusters: list[str],
        env_get: Callable[[str, str], str],
        log: Callable[[str], None],
    ) -> None:
        load_runtime_thruster_parameters(
            path=path,
            thruster_names=thruster_names,
            global_params=self.global_params,
            scale=self.scale,
            direct_scale=self.direct_scale,
            reverse_asymmetry=self.reverse_asymmetry,
            tau_up=self.tau_up,
            tau_down=self.tau_down,
            sim_profile=sim_profile,
            vertical_thrusters=vertical_thrusters,
            horizontal_thrusters=horizontal_thrusters,
            env_get=env_get,
            log=log,
        )

    def log_summary(
        self,
        *,
        perf_cfg: dict,
        all_thruster_names: list[str],
        yaw_thrusters: list[str],
        log: Callable[[str], None],
    ) -> None:
        log_thruster_runtime_summary(
            global_params=self.global_params,
            direct_scale=self.direct_scale,
            reverse_asymmetry=self.reverse_asymmetry,
            tau_up=self.tau_up,
            tau_down=self.tau_down,
            perf_cfg=perf_cfg,
            all_thruster_names=all_thruster_names,
            yaw_thrusters=yaw_thrusters,
            log=log,
        )


__all__ = ["ThrusterParameterRuntime"]
