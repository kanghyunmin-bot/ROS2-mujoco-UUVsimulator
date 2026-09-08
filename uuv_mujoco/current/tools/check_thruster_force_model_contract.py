#!/usr/bin/env python3
"""Dependency-light contract for the SITL thruster force-model selection."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    launch = (ROOT / "launch_uuv_sim.sh").read_text(encoding="utf-8")
    cli = (ROOT / "sim/runtime/cli_profile.py").read_text(encoding="utf-8")
    selector = (ROOT / "sim/physics/thruster_performance_selector.py").read_text(
        encoding="utf-8"
    )
    web_manager = (ROOT / "gui/web_process_manager.py").read_text(encoding="utf-8")
    tk_start = (ROOT / "gui/sim_stack_start_process.py").read_text(encoding="utf-8")
    debug_schedule = (ROOT / "sim/runtime/thruster_debug_schedule.py").read_text(
        encoding="utf-8"
    )
    required = {
        "measured T200 default": 'UUV_SITL_THRUSTER_FORCE_MODEL:-t200',
        "polynomial switch": 'append_extra_arg_if_missing "--disable-thruster-perf"',
        "T200 opt-in": 'append_extra_arg_if_missing "--thruster-perf-direct"',
        "invalid selector rejection": "must be polynomial or t200",
        "conflicting argument rejection": "are mutually exclusive",
    }
    for label, text in required.items():
        if text not in launch:
            raise AssertionError(f"missing {label}: {text}")
    if launch.count('append_extra_arg_if_missing "--thruster-perf-direct"') != 1:
        raise AssertionError("T200 direct mode must have exactly one opt-in site")
    if "configured polynomial/gain mapping" not in cli:
        raise AssertionError("CLI help must describe the non-curve model accurately")
    if "fixed-voltage curve disabled" not in selector:
        raise AssertionError("runtime log must make the selected force model explicit")
    if "THRUSTER_DEBUG_SAMPLE_PERIOD_S = 0.01" not in debug_schedule:
        raise AssertionError("thruster debug must preserve the 100 Hz SITL actuator cadence")
    for label, source in (("web", web_manager), ("Tk", tk_start)):
        if "UUV_MJ_THRUSTER_DEBUG_CSV" not in source or "_thrusters.csv" not in source:
            raise AssertionError(f"{label} GUI must preserve high-rate plant input automatically")
    print("thruster_force_model_contract=PASS default=t200 fallback=polynomial debug=100Hz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
