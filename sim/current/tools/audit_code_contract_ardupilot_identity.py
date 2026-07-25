"""ArduPilot source identity checks."""

from __future__ import annotations

from audit_code_contract_ardupilot_identity_checks import build_gitlink_check, build_source_tag_check
from audit_code_contract_ardupilot_identity_info import read_ardupilot_identity
from audit_code_contract_types import Check


def build_ardupilot_identity_checks() -> tuple[list[Check], dict[str, str]]:
    identity = read_ardupilot_identity()
    return [build_source_tag_check(identity), build_gitlink_check(identity)], identity.metadata()


__all__ = ["build_ardupilot_identity_checks"]
