"""Boolean source rules for active thruster conversion audits."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all


def conversion_contract_passes(
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
) -> bool:
    return (
        scheduler_contract_passes(step_callbacks_py, hydro_setup_py)
        and force_update_contract_passes(forces_py)
        and command_conversion_contract_passes(command_py)
        and wrench_accumulation_contract_passes(wrench_py)
        and force_model_contract_passes(
            force_model_py=force_model_py,
            force_performance_py=force_performance_py,
            force_polynomial_py=force_polynomial_py,
        )
        and immersion_contract_passes(immersion_py)
    )


def scheduler_contract_passes(step_callbacks_py: Path, hydro_setup_py: Path) -> bool:
    return contains_all(
        step_callbacks_py,
        [
            "hydro_runtime.thruster_scheduler.due(float(data.time), fallback_dt=float(model.opt.timestep))",
            "thruster_actuator_runtime.update_forces(dt, base_id=base_id)",
        ],
    ) and contains_all(
        hydro_setup_py,
        [
            "thruster_loop_hz = float(np.clip(args.thruster_loop_hz, 1.0, 500.0))",
            "thruster_loop_dt = 1.0 / thruster_loop_hz",
            "FixedRateSimScheduler(dt=thruster_loop_dt)",
        ],
    )


def force_update_contract_passes(forces_py: Path) -> bool:
    return contains_all(
        forces_py,
        [
            "update_thruster_state(runtime, name, dt, params)",
            "shaped_thruster_command(runtime, state_value, params)",
            "thruster_force(runtime, name, shaped_cmd)",
            "runtime.data.ctrl[aid] = force",
            "accumulate_thruster_wrench(",
        ],
    )


def command_conversion_contract_passes(command_py: Path) -> bool:
    return contains_all(
        command_py,
        [
            "runtime.state[name] = first_order_response(",
            "shape_thruster_command(",
            "force_from_shaped_command(",
            "force_immersion_scale(runtime, name)",
        ],
    )


def wrench_accumulation_contract_passes(wrench_py: Path) -> bool:
    return contains_all(
        wrench_py,
        [
            "runtime.last_force_body += force_body",
            "runtime.last_torque_body += np.cross(r_body, force_body)",
            "runtime.last_reaction_torque_world",
        ],
    )


def force_model_contract_passes(
    *,
    force_model_py: Path,
    force_performance_py: Path,
    force_polynomial_py: Path,
) -> bool:
    return contains_all(
        force_model_py,
        [
            "def force_from_shaped_command",
            "performance_curve_active(perf_cfg)",
            "force_from_polynomial_model(",
        ],
    ) and contains_all(
        force_performance_py,
        [
            "def pwm_to_force_from_performance",
            "* 400.0 + 1500.0",
            'perf_cfg.get("direct")',
            "thruster_direct_scale.get(name, 1.0)",
        ],
    ) and contains_all(
        force_polynomial_py,
        [
            "scaled_polynomial_force",
            "thruster_global[\"reverse_asymmetry\"]",
        ],
    )


def immersion_contract_passes(immersion_py: Path) -> bool:
    return contains_all(
        immersion_py,
        [
            'surface_sampler = getattr(runtime, "surface_height_sampler", None)',
            "surface_sampler(site_position.copy(), float(runtime.data.time))",
            "site_depth_m = float(surface_height - site_position[2])",
            "submerged_fraction(",
            "runtime.thruster_air_force_scale",
            "runtime.thruster_immersion_half_height_m",
        ],
    )


__all__ = [
    "command_conversion_contract_passes",
    "conversion_contract_passes",
    "force_model_contract_passes",
    "force_update_contract_passes",
    "immersion_contract_passes",
    "scheduler_contract_passes",
    "wrench_accumulation_contract_passes",
]
