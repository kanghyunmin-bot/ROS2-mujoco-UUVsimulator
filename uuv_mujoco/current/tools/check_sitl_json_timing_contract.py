#!/usr/bin/env python3
"""Smoke-check JSON timing payload modes and strict GUI estimator isolation."""

from __future__ import annotations

import os
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.sitl_json_payload import _payload_from_state  # noqa: E402
from bridge.sitl_types import VerticalEstimate  # noqa: E402
from gui.config_paths import SIM_STACK_DIR  # noqa: E402
from gui.sim_stack_env_contract import build_gui_sim_stack_env  # noqa: E402


TIMING_MODE_ENV = "ROS2_UUV_SITL_JSON_TIMING_MODE"


def _sensor_payload(env: dict[str, str]) -> dict[str, object]:
    vertical_est = VerticalEstimate(
        depth_m=0.5,
        pressure_pa=106_665.0,
        pos_ned=np.array([1.0, 2.0, 0.5], dtype=np.float64),
        vel_ned=np.array([0.1, 0.2, 0.3], dtype=np.float64),
        alt_m=-0.5,
    )
    with patch.dict(os.environ, env, clear=True):
        return _payload_from_state(
            object(),
            1.25,
            np.zeros(3, dtype=np.float64),
            np.array([0.0, 0.0, 9.80665], dtype=np.float64),
            vertical_est,
            np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            0.0,
            0.0,
            0.0,
            None,
        )


def _assert_payload_timing(
    payload: dict[str, object],
    *,
    no_time_sync: bool,
    no_lockstep: bool,
) -> None:
    if payload.get("no_time_sync") is not no_time_sync:
        raise AssertionError(
            f"no_time_sync: expected {no_time_sync!r}, "
            f"got {payload.get('no_time_sync')!r}"
        )
    if payload.get("no_lockstep") is not no_lockstep:
        raise AssertionError(
            f"no_lockstep: expected {no_lockstep!r}, "
            f"got {payload.get('no_lockstep')!r}"
        )


def check_general_default_stays_async() -> None:
    _assert_payload_timing(
        _sensor_payload({}),
        no_time_sync=True,
        no_lockstep=True,
    )


def check_diagnostic_async_is_explicitly_selectable() -> None:
    _assert_payload_timing(
        _sensor_payload({TIMING_MODE_ENV: "async"}),
        no_time_sync=True,
        no_lockstep=True,
    )


def check_lockstep_payload_enables_time_sync() -> None:
    _assert_payload_timing(
        _sensor_payload({TIMING_MODE_ENV: "lockstep"}),
        no_time_sync=False,
        no_lockstep=False,
    )


def check_strict_gui_defaults_and_truth_isolation() -> None:
    gui_source = (ROOT / "gui" / "sim_stack_env_forced_ekf.py").read_text(
        encoding="utf-8"
    )
    if '"SITL_AHRS_EKF_TYPE": "10"' in gui_source:
        raise AssertionError("strict GUI must not select the SITL truth AHRS")

    for backend in ("docker", "native"):
        env = build_gui_sim_stack_env(
            {},
            backend=backend,
            sim_stack_dir=SIM_STACK_DIR,
        )
        if env.get("SITL_AHRS_EKF_TYPE") != "3":
            raise AssertionError(f"{backend} strict GUI must select EKF3")
        if env.get(TIMING_MODE_ENV) != "lockstep":
            raise AssertionError(f"{backend} strict GUI must select lockstep timing")
        _assert_payload_timing(
            _sensor_payload(env),
            no_time_sync=False,
            no_lockstep=False,
        )


def check_gui_diagnostic_async_override_keeps_ekf3() -> None:
    for backend in ("docker", "native"):
        env = build_gui_sim_stack_env(
            {TIMING_MODE_ENV: "async"},
            backend=backend,
            sim_stack_dir=SIM_STACK_DIR,
        )
        if env.get(TIMING_MODE_ENV) != "async":
            raise AssertionError(f"{backend} GUI must preserve explicit async mode")
        if env.get("SITL_AHRS_EKF_TYPE") != "3":
            raise AssertionError(f"{backend} async diagnostic must not enable truth AHRS")
        _assert_payload_timing(
            _sensor_payload(env),
            no_time_sync=True,
            no_lockstep=True,
        )


def main() -> int:
    check_general_default_stays_async()
    check_diagnostic_async_is_explicitly_selectable()
    check_lockstep_payload_enables_time_sync()
    check_strict_gui_defaults_and_truth_isolation()
    check_gui_diagnostic_async_override_keeps_ekf3()
    print("sitl_json_timing_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
