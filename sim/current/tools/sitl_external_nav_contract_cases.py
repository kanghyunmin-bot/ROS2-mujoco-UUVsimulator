"""SITL ExternalNav contract smoke cases."""

from __future__ import annotations

import time
from types import SimpleNamespace

from bridge.sitl_external_nav_contract import _enforce_extnav_contract
from bridge.sitl_external_nav_contract_freshness import _native_vpd_fresh_enough
from sitl_external_nav_contract_fixture import assert_raises, transport


def check_required_disabled_fault_and_grace() -> None:
    _enforce_extnav_contract(transport(_sitl_extnav_required=False, _sitl_extnav_enabled=False))
    assert_raises(
        lambda: _enforce_extnav_contract(transport(_sitl_extnav_enabled=False)),
        "ExternalNav output is disabled",
    )
    assert_raises(
        lambda: _enforce_extnav_contract(transport(_sitl_extnav_fault="synthetic fault")),
        "synthetic fault",
    )
    _enforce_extnav_contract(transport(_sitl_extnav_start_wall=time.monotonic(), _sitl_extnav_last_send_wall=0.0))


def check_tx_missing_and_stale() -> None:
    assert_raises(
        lambda: _enforce_extnav_contract(transport(_sitl_extnav_last_send_wall=0.0)),
        "no VISION_POSITION_DELTA",
    )
    assert_raises(
        lambda: _enforce_extnav_contract(transport(_sitl_extnav_last_send_wall=1.0e-9, _sitl_extnav_max_stale_s=0.0)),
        "ExternalNav TX stale",
    )


def check_native_vpd_freshness() -> None:
    native_before_start = transport(
        _native_vpd_events=[SimpleNamespace(t_replay_s=5.0)],
        _sensor_replay_ready=True,
        _sensor_replay_current_t_s=4.0,
    )
    if not _native_vpd_fresh_enough(native_before_start):
        raise AssertionError("native VPD before first event should be fresh")

    native_stale = transport(
        _native_vpd_events=[SimpleNamespace(t_replay_s=5.0)],
        _sensor_replay_ready=True,
        _sensor_replay_current_t_s=6.0,
        _native_vpd_last_replay_t_s=5.0,
        _sitl_extnav_max_stale_s=0.5,
    )
    if _native_vpd_fresh_enough(native_stale):
        raise AssertionError("native VPD stale replay sample should not be fresh")


__all__ = [
    "check_native_vpd_freshness",
    "check_required_disabled_fault_and_grace",
    "check_tx_missing_and_stale",
]
