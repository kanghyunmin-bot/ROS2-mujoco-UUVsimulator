"""Build active-runtime identity source-audit checks."""

from __future__ import annotations

from audit_code_contract_runtime_identity_eval import evaluate_runtime_identity
from audit_code_contract_runtime_identity_inputs import collect_runtime_identity_inputs
from audit_code_contract_runtime_identity_report import (
    active_runtime_identity_evidence,
    active_runtime_identity_metadata,
)
from audit_code_contract_types import Check


def build_active_runtime_alias_check() -> tuple[Check, dict[str, str]]:
    inputs = collect_runtime_identity_inputs()
    result = evaluate_runtime_identity(inputs)
    check = Check(
        check_id="active_runtime_alias_current",
        status=str(result["status"]),
        title="Active runtime resolves through sim/current",
        conclusion=(
            "Live launch, GUI, setup, and validation paths must resolve the active runtime "
            "through sim/current. The v2.2 directory name is only the compatibility backing "
            "directory until a physical rename is done."
        ),
        evidence=active_runtime_identity_evidence(inputs, result),
        official_refs=[],
    )
    return check, active_runtime_identity_metadata(inputs)
