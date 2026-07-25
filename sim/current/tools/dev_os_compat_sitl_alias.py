"""Active-runtime alias checks for development OS compatibility."""

from __future__ import annotations

import os

from dev_os_compat_common import WORKSPACE, CheckResult


def check_active_runtime_alias(results: list[CheckResult]) -> None:
    active_alias = WORKSPACE / "sim" / "current"
    alias_target = os.readlink(active_alias) if active_alias.is_symlink() else None
    if alias_target == "v2.2":
        results.append(
            CheckResult(
                "active_runtime_alias",
                "pass",
                "sim/current is the active runtime; v2.2 is only the compatibility backing directory",
            )
        )
        return
    if active_alias.exists():
        detail = f"sim/current exists but points to {alias_target or active_alias.resolve()}"
        results.append(CheckResult("active_runtime_alias", "warn", detail))
        return
    results.append(CheckResult("active_runtime_alias", "warn", "sim/current alias is missing"))


__all__ = ["check_active_runtime_alias"]
