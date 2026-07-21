"""Temporary-file helpers for thruster parameter loader smoke checks."""

from __future__ import annotations

import json
import pathlib

from sim.physics.thruster_param_loader import load_thruster_params_file

from thruster_param_loader_smoke_fixture import thruster_param_payload


def load_thruster_smoke_payload(path: pathlib.Path, maps: dict[str, dict]) -> bool:
    path.write_text(json.dumps(thruster_param_payload()), encoding="utf-8")
    return load_thruster_params_file(
        path,
        ["t1", "t2"],
        thruster_global=maps["global"],
        thruster_scale=maps["scale"],
        thruster_direct_scale=maps["direct"],
        thruster_reverse_asymmetry=maps["reverse"],
        thruster_tau_up=maps["tau_up"],
        thruster_tau_down=maps["tau_down"],
    )


__all__ = ["load_thruster_smoke_payload"]
