"""Fixtures for SITL ExternalNav contract smoke checks."""

from __future__ import annotations

import time
from types import SimpleNamespace


def transport(**overrides):
    now = time.monotonic()
    values = dict(
        _sitl_extnav_required=True,
        _sitl_extnav_enabled=True,
        _sitl_extnav_fault="",
        _sitl_extnav_start_wall=now - 10.0,
        _sitl_extnav_grace_s=1.0,
        _sitl_extnav_last_send_wall=now,
        _sitl_extnav_max_stale_s=0.5,
        _native_vpd_events=[],
        _sensor_replay_ready=False,
        _sensor_replay_current_t_s=None,
        _native_vpd_last_replay_t_s=None,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def assert_raises(fn, text: str) -> None:
    try:
        fn()
    except RuntimeError as exc:
        if text not in str(exc):
            raise AssertionError(f"expected {text!r} in {exc!r}") from exc
        return
    raise AssertionError(f"expected RuntimeError containing {text!r}")


__all__ = ["assert_raises", "transport"]
