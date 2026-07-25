"""Result builder for actuator contract wrench audits."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from actuator_wrench_common import AXES, FLU_TO_FRD, load_json
from actuator_wrench_axis import wrench_for_axis
from actuator_wrench_model import (
    apply_profile_direct_gain_scales,
    apply_runtime_site_adjustments,
    load_direct_gains,
)
from actuator_wrench_summary import coupling_summary


def build_actuator_wrench_result(
    *,
    scene: Path,
    profile_file: Path,
    profile_name: str,
    thruster_params: Path,
    horizontal_z_offset: float | None,
    vertical_x_scale: float | None,
    unit_gains: bool,
) -> dict[str, Any]:
    import mujoco

    profiles = load_json(profile_file)
    profile = profiles.get(profile_name, {})
    if not isinstance(profile, dict):
        raise SystemExit(f"profile {profile_name!r} is not a JSON object in {profile_file}")

    model = mujoco.MjModel.from_xml_path(str(scene))
    apply_runtime_site_adjustments(
        model,
        profile,
        horizontal_z_offset=horizontal_z_offset,
        vertical_x_scale=vertical_x_scale,
    )
    gains = apply_profile_direct_gain_scales(load_direct_gains(thruster_params), profile)
    result: dict[str, Any] = {
        "scene": str(scene),
        "profile_file": str(profile_file),
        "profile_name": str(profile_name),
        "thruster_params": str(thruster_params),
        "unit_gains": bool(unit_gains),
        "direct_gains": gains,
        "axes": {},
    }
    for axis_idx, axis in enumerate(AXES):
        force_flu, torque_flu, per_thruster = wrench_for_axis(
            model,
            axis_idx,
            gains,
            unit_gains=bool(unit_gains),
        )
        force_frd = FLU_TO_FRD @ force_flu
        torque_frd = FLU_TO_FRD @ torque_flu
        result["axes"][axis] = {
            "force_flu": force_flu.tolist(),
            "torque_flu": torque_flu.tolist(),
            "force_frd": force_frd.tolist(),
            "torque_frd": torque_frd.tolist(),
            "summary": coupling_summary(force_frd, torque_frd, axis),
            "per_thruster": per_thruster,
        }
    return result


__all__ = [
    "build_actuator_wrench_result",
    "coupling_summary",
    "wrench_for_axis",
]
