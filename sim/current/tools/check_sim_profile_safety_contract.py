#!/usr/bin/env python3
"""Regression checks for one-source physics defaults and tuning guardrails."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.config_physics import PHYSICS_PARAM_SPECS  # noqa: E402
from gui.web_tool_files import _parse_web_physics_value  # noqa: E402
from physics.sim_profile_defaults import DEFAULT_SIM_PROFILES  # noqa: E402
from physics.sim_profile_helpers import build_sim_profile  # noqa: E402
from physics.sim_profile_validation import validate_profile_param_value  # noqa: E402
from sim.physics.thruster_param_defaults import (  # noqa: E402
    default_thruster_global_params,
    default_thruster_params_payload,
)


def _physics_only(value):
    if isinstance(value, dict):
        return {
            key: _physics_only(item)
            for key, item in value.items()
            if key not in {"note", "source"} and not str(key).endswith("_note")
        }
    if isinstance(value, list):
        return [_physics_only(item) for item in value]
    return value


def _equivalent(left, right) -> bool:
    if isinstance(left, dict) and isinstance(right, dict):
        return left.keys() == right.keys() and all(_equivalent(left[key], right[key]) for key in left)
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(_equivalent(a, b) for a, b in zip(left, right))
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1.0e-12)
    return left == right


def _expect_rejected(key: str, value) -> None:
    try:
        validate_profile_param_value(key, value)
    except (TypeError, ValueError):
        return
    raise AssertionError(f"unsafe value accepted: {key}={value!r}")


def _expect_profile_rejected(profile: dict, label: str) -> None:
    try:
        build_sim_profile({"current": profile}, "current")
    except ValueError:
        return
    raise AssertionError(f"unsafe profile accepted: {label}")


def _check_profile_default_parity() -> None:
    configured = json.loads((ROOT / "config" / "sim_profiles.json").read_text())["current"]
    fallback = DEFAULT_SIM_PROFILES["current"]
    core_keys = (
        "body_components",
        "body_inertia_scale_xyz",
        "buoyancy_model",
        "buoyancy_point_blend",
        "buoyancy_points",
        "buoyancy_scale",
        "buoyancy_slope_scale",
        "cob_torque_scale",
        "cob_x_offset",
        "cob_z_offset",
        "current_world",
        "ellipsoid_model",
        "fossen_residual_hydro",
        "hydrostatic_restoring",
        "hydrostatic_volume_source",
        "mujoco_fluid_geom_size_scales",
        "mujoco_fluidcoef_immersion_scale",
        "mujoco_fluidcoef_geom_scales",
        "mujoco_fluidcoef_scale",
        "thruster_force_max",
        "thruster_voltage",
        "yaw_torque_scale",
    )
    for key in core_keys:
        if not _equivalent(_physics_only(fallback[key]), _physics_only(configured[key])):
            raise AssertionError(f"runtime fallback differs from current profile: {key}")

    gui_defaults = {str(spec["key"]): spec["default"] for spec in PHYSICS_PARAM_SPECS}
    for key, value in gui_defaults.items():
        if value != fallback[key]:
            raise AssertionError(f"GUI fallback differs from runtime fallback: {key}")


def _check_thruster_default_parity() -> None:
    configured = json.loads((ROOT / "config" / "thruster_params.json").read_text())
    fallback_global = default_thruster_global_params()
    for key, value in fallback_global.items():
        if configured["global"].get(key) != value:
            raise AssertionError(f"thruster global fallback differs: {key}")

    fallback = default_thruster_params_payload(configured["per_thruster"])
    for name in configured["per_thruster"]:
        for key in ("gain_scale", "direct_gain_scale", "reverse_asymmetry", "tau_up", "tau_down"):
            expected = configured["per_thruster"][name].get(key)
            actual = fallback["per_thruster"][name][key]
            if actual != expected:
                raise AssertionError(f"thruster fallback differs: {name}.{key}")


def _check_guardrails() -> None:
    accepted = DEFAULT_SIM_PROFILES["current"]
    build_sim_profile({"current": accepted}, "current")
    _expect_rejected("buoyancy_scale", 1.1)
    _expect_rejected("cob_x_offset", 1.0)
    _expect_rejected("current_world", [0.0, float("nan"), 0.0])
    _expect_rejected("current_world", [0.0, 4.0, 0.0])
    _expect_rejected("current_world", [0.0, 0.0])
    _expect_rejected("buoyancy_scale", [1.0])
    _expect_rejected("buoyancy_scale", True)
    _expect_rejected("current_world", [0.0, False, 0.0])
    bad_boolean = dict(accepted)
    bad_boolean["mujoco_fluidcoef_immersion_scale"] = "false"
    try:
        build_sim_profile({"current": bad_boolean}, "current")
    except ValueError:
        pass
    else:
        raise AssertionError("non-boolean immersion scale accepted")
    _expect_rejected("body_inertia_scale_xyz", [0.0, 1.0, 1.0])

    matching_aliases = deepcopy(accepted)
    matching_aliases["water_current_world"] = list(matching_aliases["current_world"])
    build_sim_profile({"current": matching_aliases}, "current")
    conflicting_aliases = deepcopy(matching_aliases)
    conflicting_aliases["water_current_world"] = [0.1, 0.0, 0.0]
    _expect_profile_rejected(conflicting_aliases, "conflicting water-current aliases")

    malformed = deepcopy(accepted)
    malformed["thruster_voltage"] = True
    _expect_profile_rejected(malformed, "boolean finite scalar")

    malformed = deepcopy(accepted)
    malformed["linear_damping_diag"] = [1.0] * 5
    _expect_profile_rejected(malformed, "short 6DOF damping vector")
    malformed = deepcopy(accepted)
    malformed["added_mass_diag"] = [0.0, 0.0, True, 0.0, 0.0, 0.0]
    _expect_profile_rejected(malformed, "boolean in 6DOF vector")

    malformed = deepcopy(accepted)
    malformed["hydrostatic_restoring"] = []
    _expect_profile_rejected(malformed, "non-object hydrostatic restoring")
    malformed = deepcopy(accepted)
    malformed["hydrostatic_restoring"]["active"] = 1
    _expect_profile_rejected(malformed, "numeric hydrostatic active flag")
    malformed = deepcopy(accepted)
    malformed["hydrostatic_restoring"]["roll_stiffness_nm_per_rad"] = True
    _expect_profile_rejected(malformed, "boolean hydrostatic stiffness")
    malformed = deepcopy(accepted)
    malformed["hydrostatic_restoring"]["pitch_trim_rad"] = float("nan")
    _expect_profile_rejected(malformed, "non-finite hydrostatic trim")

    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"] = "enabled"
    _expect_profile_rejected(malformed, "non-object Fossen payload")
    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"]["active"] = "true"
    _expect_profile_rejected(malformed, "string Fossen active flag")
    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"]["linear"] = []
    _expect_profile_rejected(malformed, "non-object Fossen coefficient group")
    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"]["linear"]["y_v"] = float("inf")
    _expect_profile_rejected(malformed, "non-finite Fossen coefficient")
    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"]["quadratic"]["n_abs_r_r"] = False
    _expect_profile_rejected(malformed, "boolean Fossen coefficient")
    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"]["added_mass"]["active"] = 1
    _expect_profile_rejected(malformed, "numeric Fossen added-mass active flag")
    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"]["added_mass_matrix"] = [[0.0] * 6 for _ in range(5)]
    _expect_profile_rejected(malformed, "wrong-shape Fossen added-mass matrix")
    malformed = deepcopy(accepted)
    malformed["fossen_residual_hydro"]["added_mass_matrix"] = [
        [False if (row == 0 and column == 0) else 0.0 for column in range(6)]
        for row in range(6)
    ]
    _expect_profile_rejected(malformed, "boolean Fossen matrix coefficient")

    # Inactive/documentation-only extensions and explicitly zero active terms
    # remain legal; validation targets fields consumed by the current runtime.
    extension_profile = deepcopy(accepted)
    extension_profile["hydrostatic_restoring"] = {
        "active": False,
        "source": "disabled calibration extension",
    }
    extension_profile["fossen_residual_hydro"] = {
        "active": False,
        "note": "reserved extension",
        "future_payload": {"format": "not consumed"},
    }
    build_sim_profile({"current": extension_profile}, "current")
    zero_profile = deepcopy(accepted)
    zero_profile["fossen_residual_hydro"] = {
        "active": True,
        "linear": {"y_v": 0.0, "z_w": 0.0},
        "added_mass": {"active": True, "x_u": 0.0},
    }
    build_sim_profile({"current": zero_profile}, "current")

    for key, raw in (("buoyancy_scale", "1.1"), ("cob_x_offset", "1.0")):
        try:
            _parse_web_physics_value({"key": key, "label": key}, raw)
        except ValueError:
            continue
        raise AssertionError(f"web physics endpoint accepted unsafe value: {key}={raw}")


def main() -> int:
    _check_profile_default_parity()
    _check_thruster_default_parity()
    _check_guardrails()
    print("sim_profile_safety_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
