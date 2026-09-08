#!/usr/bin/env python3
"""Regression checks for Ground Truth isolation from parity ExternalNav."""

from __future__ import annotations

import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.sitl_transport_extnav_base import initialize_extnav_base_state  # noqa: E402
from bridge.sitl_transport_extnav_runtime import initialize_extnav_runtime_state  # noqa: E402


def _transport(*, truth_allowed: bool) -> SimpleNamespace:
    return SimpleNamespace(_sitl_truth_extnav_allowed=truth_allowed)


def check_parity_blocks_truth_external_nav() -> None:
    env = {
        "ROS2_UUV_SITL_EXTNAV_ENABLE": "1",
        "ROS2_UUV_REQUIRE_EXTNAV_TX": "1",
        "ROS2_UUV_SITL_BRIDGE_EXTNAV_DISABLE": "0",
    }
    with patch.dict(os.environ, env, clear=False):
        transport = _transport(truth_allowed=False)
        initialize_extnav_base_state(transport)
        initialize_extnav_runtime_state(transport)

    assert transport._sitl_truth_extnav_blocked
    assert not transport._sitl_extnav_enabled
    assert not transport._sitl_extnav_required


def check_diagnostic_mode_can_opt_in() -> None:
    env = {
        "ROS2_UUV_SITL_EXTNAV_ENABLE": "1",
        "ROS2_UUV_REQUIRE_EXTNAV_TX": "1",
        "ROS2_UUV_SITL_BRIDGE_EXTNAV_DISABLE": "0",
    }
    with patch.dict(os.environ, env, clear=False):
        transport = _transport(truth_allowed=True)
        initialize_extnav_base_state(transport)
        initialize_extnav_runtime_state(transport)

    assert not transport._sitl_truth_extnav_blocked
    assert transport._sitl_extnav_enabled
    assert transport._sitl_extnav_required


def check_explicit_bridge_disable_wins() -> None:
    env = {
        "ROS2_UUV_SITL_EXTNAV_ENABLE": "1",
        "ROS2_UUV_REQUIRE_EXTNAV_TX": "1",
        "ROS2_UUV_SITL_BRIDGE_EXTNAV_DISABLE": "1",
    }
    with patch.dict(os.environ, env, clear=False):
        transport = _transport(truth_allowed=True)
        initialize_extnav_base_state(transport)
        initialize_extnav_runtime_state(transport)

    assert not transport._sitl_extnav_enabled
    assert transport._sitl_extnav_required


def main() -> int:
    check_parity_blocks_truth_external_nav()
    check_diagnostic_mode_can_opt_in()
    check_explicit_bridge_disable_wins()
    print("truth_extnav_isolation=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
