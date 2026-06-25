"""Integrated active-runtime flow contract source check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_runtime_flow_coupling_contract_check(runtime_paths: dict[str, Path]) -> Check:
    step_runtime_py = runtime_paths["simulation_step_runtime_py"]
    raw_pwm_py = runtime_paths["simulation_step_raw_pwm_py"]
    step_physics_py = runtime_paths["simulation_step_physics_py"]
    underwater_py = runtime_paths["underwater_wrench_runtime_py"]
    ok = (
        contains_all(
            step_runtime_py,
            [
                "self.spin_ros_once()",
                "thruster_due, thruster_dt = self.thruster_update_due()",
                "return self.run_raw_pwm_step(is_paused, publish_ros, thruster_due, thruster_dt)",
            ],
        )
        and contains_all(
            raw_pwm_py,
            [
                "now = time.monotonic()",
                "runtime.sitl_servo_runtime.apply_to_targets(",
                "apply_common_step_physics(",
            ],
        )
        and contains_all(
            step_physics_py,
            [
                "owner.update_thruster_forces(thr_dt)",
                "owner.apply_underwater_wrench(owner.model.opt.timestep if not is_paused else 0.0)",
                "owner.mujoco.mj_step(owner.model, owner.data)",
            ],
        )
        and contains_all(
            underwater_py,
            [
                "rel_lin_vel_body = lin_vel_body - current_body",
                "self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)",
                "apply_hydrodynamic_wrenches(",
            ],
        )
    )
    return Check(
        check_id="active_runtime_integrated_flow_contract",
        status="PASS" if ok else "FAIL",
        title="Runtime order keeps RC/PWM, thrusters, fluidcoef, and MuJoCo step separated",
        conclusion=(
            "Each tick spins ROS/transport first, applies raw PWM or direct command targets, updates thruster "
            "force on the thruster cadence, updates dynamic fluidcoef inside the underwater wrench phase, "
            "then advances MuJoCo. This prevents sensor publishing, RC input, actuator conversion, and "
            "dynamic ellipsoid tuning from silently owning the same state surface."
        ),
        evidence=[
            evidence(step_runtime_py, "self.spin_ros_once()"),
            evidence(raw_pwm_py, "runtime.sitl_servo_runtime.apply_to_targets("),
            evidence(step_physics_py, "owner.update_thruster_forces(thr_dt)"),
            evidence(step_physics_py, "owner.mujoco.mj_step(owner.model, owner.data)"),
            evidence(underwater_py, "self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)"),
        ],
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"], OFFICIAL_REFS["mujoco_fluid"]],
    )


__all__ = ["build_runtime_flow_coupling_contract_check"]
