#!/usr/bin/env python3
"""Smoke check for MuJoCo model runtime bootstrap contracts."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.model_runtime_setup import load_model_runtime_setup  # noqa: E402
from sim.runtime.parsing import to_float_array, to_float_matrix  # noqa: E402


def _env_float(_name: str, default: float) -> float:
    return float(default)


def _env_flag(_name: str, default: bool = False) -> bool:
    return bool(default)


def _env_get(_name: str, default: str | None = None) -> str | None:
    return default


def _load_current_profile() -> dict:
    payload = json.loads((ROOT / "config" / "sim_profiles.json").read_text(encoding="utf-8"))
    return dict(payload["current"])


def main() -> int:
    import mujoco

    args = SimpleNamespace(scene=str(ROOT / "scenes" / "tank_current_scene.xml"), fluid_model="current")
    setup = load_model_runtime_setup(
        args=args,
        mujoco_module=mujoco,
        sim_profile=_load_current_profile(),
        run_mode="closed_loop",
        env_float=_env_float,
        env_flag=_env_flag,
        env_get=_env_get,
        to_float_array=to_float_array,
        to_float_matrix=to_float_matrix,
    )
    if not isinstance(setup.fluid_geom_names, dict):
        raise AssertionError(f"fluid_geom_names must be dict[int,str], got {type(setup.fluid_geom_names)!r}")
    if not setup.fluid_geom_ids:
        raise AssertionError("expected at least one fluid geom")
    missing = [geom_id for geom_id in setup.fluid_geom_ids if int(geom_id) not in setup.fluid_geom_names]
    if missing:
        raise AssertionError(f"fluid geom names missing ids: {missing}")
    if np.asarray(setup.fluidcoef_dynamic_setup.base).shape[-1] != 5:
        raise AssertionError("dynamic fluidcoef base rows must have 5 coefficients")
    print("model_runtime_setup=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
