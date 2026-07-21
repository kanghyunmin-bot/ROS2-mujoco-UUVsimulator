#!/usr/bin/env python3
"""Smoke checks for thruster parameter JSON loading."""

from __future__ import annotations

import pathlib
import sys
import json


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
TOOLS_DIR = pathlib.Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from thruster_param_loader_smoke_cases import run_thruster_param_loader_smoke  # noqa: E402


YAW_THRUSTERS = ("yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr")
VERTICAL_THRUSTERS = ("ver_lf", "ver_lr", "ver_rf", "ver_rr")


def check_qgc_default_thruster_contract() -> None:
    path = ROOT / "config" / "thruster_params.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    global_params = payload.get("global", {})
    per_thruster = payload.get("per_thruster", {})

    tau_up = float(global_params.get("tau_up", 999.0))
    tau_down = float(global_params.get("tau_down", 999.0))
    if tau_up > 0.05 or tau_down > 0.07:
        raise AssertionError(f"QGC RC response tau too slow: tau_up={tau_up}, tau_down={tau_down}")

    yaw_gains = []
    for name in YAW_THRUSTERS:
        cfg = per_thruster.get(name, {})
        yaw_gains.append(float(cfg.get("gain_scale", 0.0)))
        if "tau_up" in cfg or "tau_down" in cfg:
            raise AssertionError(f"{name} must not override QGC yaw tau with slow per-thruster dynamics")
        if "reverse_asymmetry" in cfg:
            raise AssertionError(f"{name} must not override QGC yaw reverse asymmetry")
    if max(yaw_gains) - min(yaw_gains) > 1.0e-9:
        raise AssertionError(f"QGC yaw gain must be symmetric: {yaw_gains}")

    for name in VERTICAL_THRUSTERS:
        cfg = per_thruster.get(name, {})
        if "tau_up" in cfg or "tau_down" in cfg:
            raise AssertionError(f"{name} must use global QGC heave tau")


def main() -> int:
    run_thruster_param_loader_smoke()
    check_qgc_default_thruster_contract()
    print("thruster_param_loader=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
