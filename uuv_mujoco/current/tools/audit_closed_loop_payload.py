"""Payload assembly for closed-loop contract audits."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from audit_closed_loop_params import parse_param_file, watched_param_report
from audit_closed_loop_profile import (
    CURRENT_INACTIVE_KEYS,
    active_profile_report,
    load_profile,
    resolve_active_runtime,
    selected_thruster_curve,
)
from audit_closed_loop_sitl_params import load_sitl_param_contract


def build_contract_payload(
    *,
    workspace: Path,
    profile_name: str,
    sitl_log: Path | None = None,
) -> dict[str, Any]:
    sim_dir, runtime_source = resolve_active_runtime(workspace)
    real_params = parse_param_file(workspace / "real_robot.param")
    sitl_params, sitl_param_sources = load_sitl_param_contract(workspace, sim_dir, sitl_log)

    profile = load_profile(sim_dir / "config" / "sim_profiles.json", profile_name)
    requested_voltage = float(profile.get("thruster_voltage", 22.2))
    curve = selected_thruster_curve(sim_dir / "config" / "thruster_performance.json", requested_voltage)
    watched_real, watched_sitl, mismatches, missing_sitl = watched_param_report(real_params, sitl_params)
    current_profile, active_keys = active_profile_report(profile)

    return {
        "workspace": str(workspace),
        "profile": profile_name,
        "active_runtime": {
            "path": str(sim_dir),
            "source": runtime_source,
        },
        "real_robot_param": str(workspace / "real_robot.param"),
        "sitl_param": str(sim_dir / "config" / "ardusub_realrobot_contract.param"),
        "sitl_eeprom_param": str(workspace / "ardupilot" / "mav.parm"),
        "sitl_param_sources": sitl_param_sources,
        "watched_real_params": watched_real,
        "watched_sitl_params": watched_sitl,
        "real_vs_sitl_mismatches": mismatches,
        "missing_sitl_params": missing_sitl,
        "current_mode": {
            "active_profile_keys": active_keys,
            "inactive_profile_keys": list(CURRENT_INACTIVE_KEYS),
            "profile_values": current_profile,
        },
        "thruster_performance_curve": curve,
        "notes": [
            "Current MuJoCo mode uses built-in geom fluidcoef plus hydrostatic/CoB terms.",
            "Old Python 6DOF hydrodynamics runtime paths have been removed.",
            "When the T200 performance curve is active, profile thruster_force_max is not the actuator force limit.",
            "real_vs_sitl_mismatches only compares parameters observed in both sources; missing_sitl_params must be resolved before claiming full QGC/SITL parity.",
        ],
    }

__all__ = ["build_contract_payload", "load_sitl_param_contract"]
