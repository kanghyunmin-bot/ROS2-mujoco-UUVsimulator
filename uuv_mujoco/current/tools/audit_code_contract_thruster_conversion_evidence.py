"""Evidence collection for active thruster conversion source audits."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import evidence
from audit_code_contract_types import Evidence


def conversion_evidence(
    *,
    step_callbacks_py: Path,
    hydro_setup_py: Path,
    command_py: Path,
    forces_py: Path,
    force_model_py: Path,
    force_performance_py: Path,
    force_polynomial_py: Path,
    immersion_py: Path,
    wrench_py: Path,
) -> list[Evidence]:
    return [
        evidence(step_callbacks_py, "hydro_runtime.thruster_scheduler.due"),
        evidence(hydro_setup_py, "FixedRateSimScheduler(dt=thruster_loop_dt)"),
        evidence(command_py, "runtime.state[name] = first_order_response("),
        evidence(forces_py, "runtime.data.ctrl[aid] = force"),
        evidence(wrench_py, "runtime.last_reaction_torque_world"),
        evidence(force_model_py, "def force_from_shaped_command"),
        evidence(force_performance_py, "def pwm_to_force_from_performance"),
        evidence(force_polynomial_py, "scaled_polynomial_force"),
        evidence(immersion_py, "site_depth_m = float(runtime.water_surface_z - site_z)"),
    ]


__all__ = ["conversion_evidence"]
