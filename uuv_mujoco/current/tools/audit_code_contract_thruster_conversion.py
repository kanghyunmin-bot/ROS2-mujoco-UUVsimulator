"""Active thruster conversion check for source-level contract audits."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_thruster_conversion_evidence import conversion_evidence
from audit_code_contract_thruster_conversion_rules import conversion_contract_passes
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_active_runtime_thruster_conversion_check(runtime_paths: dict[str, Path]) -> Check:
    step_callbacks_py = runtime_paths["physics_step_thruster_callbacks_py"]
    hydro_setup_py = runtime_paths["hydrodynamics_runtime_setup_py"]
    command_py = runtime_paths["thruster_actuator_command_py"]
    forces_py = runtime_paths["thruster_actuator_forces_py"]
    force_model_py = runtime_paths["thruster_force_model_py"]
    force_performance_py = runtime_paths["thruster_force_performance_py"]
    force_polynomial_py = runtime_paths["thruster_force_polynomial_py"]
    immersion_py = runtime_paths["thruster_actuator_immersion_py"]
    wrench_py = runtime_paths["thruster_actuator_wrench_py"]
    ok = conversion_contract_passes(
        step_callbacks_py=step_callbacks_py,
        hydro_setup_py=hydro_setup_py,
        command_py=command_py,
        forces_py=forces_py,
        force_model_py=force_model_py,
        force_performance_py=force_performance_py,
        force_polynomial_py=force_polynomial_py,
        immersion_py=immersion_py,
        wrench_py=wrench_py,
    )
    return Check(
        check_id="active_runtime_thruster_conversion_contract",
        status="PASS" if ok else "FAIL",
        title="Thruster conversion applies timing, motor lag, force curve, and immersion once",
        conclusion=(
            "The active runtime schedules thruster updates in sim time, then converts raw normalized targets through "
            "one first-order actuator state, one T200/direct or polynomial force model, and one water-immersion scale "
            "before writing MuJoCo actuator ctrl."
        ),
        evidence=conversion_evidence(
            step_callbacks_py=step_callbacks_py,
            hydro_setup_py=hydro_setup_py,
            command_py=command_py,
            forces_py=forces_py,
            force_model_py=force_model_py,
            force_performance_py=force_performance_py,
            force_polynomial_py=force_polynomial_py,
            immersion_py=immersion_py,
            wrench_py=wrench_py,
        ),
        official_refs=[OFFICIAL_REFS["mujoco_fluid"]],
    )


__all__ = ["build_active_runtime_thruster_conversion_check"]
