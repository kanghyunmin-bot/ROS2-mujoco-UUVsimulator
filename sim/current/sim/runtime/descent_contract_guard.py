"""Runtime descent diagnostics for SITL/MuJoCo contract checks."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Sequence

from sim.runtime.descent_contract_guard_eval import build_descent_contract_message, descent_guard_should_check


@dataclass
class DescentContractGuard:
    enabled: bool
    fail_fast: bool
    vz_down_mps: float
    start_s: float
    last_warn_wall: float = -1.0

    def check(
        self,
        *,
        initial_depth_hold_active: bool,
        sim_time: float,
        base_depth_m: float,
        base_vz_down_mps: float,
        vertical_pwm: Sequence[int],
        thruster_force_world_z: float,
        buoy_force_world_z: float,
        vehicle_mass: float,
        gravity: float,
    ) -> str | None:
        if not descent_guard_should_check(
            enabled=self.enabled,
            initial_depth_hold_active=initial_depth_hold_active,
            sim_time=sim_time,
            start_s=self.start_s,
            base_vz_down_mps=base_vz_down_mps,
            vz_down_mps=self.vz_down_mps,
        ):
            return None

        now_wall = time.monotonic()
        if now_wall - self.last_warn_wall < 1.0:
            return None
        self.last_warn_wall = now_wall
        message = build_descent_contract_message(
            base_depth_m=base_depth_m,
            base_vz_down_mps=base_vz_down_mps,
            vertical_pwm=vertical_pwm,
            thruster_force_world_z=thruster_force_world_z,
            buoy_force_world_z=buoy_force_world_z,
            vehicle_mass=vehicle_mass,
            gravity=gravity,
        )
        if self.fail_fast:
            raise RuntimeError(message)
        return message
