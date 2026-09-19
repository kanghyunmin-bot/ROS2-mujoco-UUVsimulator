# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Exercise profile response priors through the actual thruster file loader."""

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.runtime.thruster_param_runtime_loader import load_runtime_thruster_parameters  # noqa: E402


def load(profile, env=None):
    maps = {key: {} for key in ["global", "scale", "direct", "reverse", "up", "down"]}
    load_runtime_thruster_parameters(
        path=ROOT / "config/thruster_params.json",
        thruster_names=["ver_lf", "yaw_lf", "yaw_lr"],
        global_params=maps["global"],
        scale=maps["scale"],
        direct_scale=maps["direct"],
        reverse_asymmetry=maps["reverse"],
        tau_up=maps["up"],
        tau_down=maps["down"],
        sim_profile=profile,
        vertical_thrusters=["ver_lf"],
        horizontal_thrusters=["yaw_lf", "yaw_lr"],
        env_get=(env or {}).get,
        log=lambda _: None,
    )
    return maps


def profile(up=0.01, down=0.015):
    return {
        "thruster_response_time_constants": {
            "calibration_status": "effective_fit",
            "provenance": "test fixture",
            "horizontal": {"tau_up_s": up, "tau_down_s": down},
        }
    }


def test_profile_applies_to_horizontal_and_preserves_file_and_vertical():
    result = load(profile())
    assert result["up"]["yaw_lf"] == 0.01
    assert result["down"]["yaw_lr"] == 0.015
    assert result["up"]["ver_lf"] is None
    assert result["global"]["tau_up"] == 0.04
    assert result["global"]["tau_down"] == 0.06
    baseline = load({})
    assert baseline["up"]["yaw_lf"] is None


def test_explicit_environment_overrides_profile():
    result = load(profile(), {"UUV_YAW_THRUSTER_TAU_UP": ".02"})
    assert result["up"]["yaw_lf"] == 0.02
    assert result["down"]["yaw_lf"] == 0.015


@pytest.mark.parametrize("bad", [0, -1, float("nan"), float("inf")])
def test_invalid_response_prior_is_rejected(bad):
    with pytest.raises(ValueError):
        load(profile(up=bad))
