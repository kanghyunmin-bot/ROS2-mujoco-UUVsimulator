"""Case runner for thruster parameter loader smoke checks."""

from __future__ import annotations

import pathlib
import tempfile

from thruster_param_loader_smoke_assertions import assert_loaded_thruster_param_maps
from thruster_param_loader_smoke_fixture import thruster_param_maps
from thruster_param_loader_smoke_io import load_thruster_smoke_payload


def run_thruster_param_loader_smoke() -> None:
    maps = thruster_param_maps()
    with tempfile.TemporaryDirectory() as tmp:
        path = pathlib.Path(tmp) / "thrusters.json"
        changed = load_thruster_smoke_payload(path, maps)
    if not changed:
        raise AssertionError("expected changed thruster params")
    assert_loaded_thruster_param_maps(maps)


__all__ = ["run_thruster_param_loader_smoke"]
