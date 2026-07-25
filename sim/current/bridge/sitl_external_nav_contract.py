"""ExternalNav required-output and stale-rate contract checks."""

from __future__ import annotations

import time

from .sitl_external_nav_contract_errors import (
    _inside_extnav_grace,
    _raise_if_extnav_disabled_or_faulted,
    _raise_if_extnav_never_sent,
    _raise_if_extnav_stale,
)
from .sitl_external_nav_contract_freshness import _native_vpd_fresh_enough


def _enforce_extnav_contract(self) -> None:
    if not self._sitl_extnav_required:
        return
    now_wall = time.monotonic()
    _raise_if_extnav_disabled_or_faulted(self)
    if _native_vpd_fresh_enough(self):
        return
    if _inside_extnav_grace(self, now_wall):
        return
    _raise_if_extnav_never_sent(self)
    _raise_if_extnav_stale(self, now_wall)


__all__ = ["_enforce_extnav_contract"]
