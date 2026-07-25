"""Source identity check facade for source-level contract audits."""

from __future__ import annotations

from audit_code_contract_ardupilot_identity import build_ardupilot_identity_checks
from audit_code_contract_runtime_identity import build_active_runtime_alias_check
from audit_code_contract_types import Check


def build_source_identity_checks() -> tuple[list[Check], dict[str, str]]:
    active_runtime_check, active_runtime_metadata = build_active_runtime_alias_check()
    ardupilot_checks, ardupilot_metadata = build_ardupilot_identity_checks()
    checks = [active_runtime_check, *ardupilot_checks]
    metadata = {**ardupilot_metadata, **active_runtime_metadata}
    return checks, metadata


__all__ = ["build_source_identity_checks"]
