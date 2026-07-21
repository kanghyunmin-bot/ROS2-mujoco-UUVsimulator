"""Top-level matrix gate for source-level runtime contracts."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import rel
from audit_code_contract_matrix_domains import CONTRACT_DOMAINS
from audit_code_contract_matrix_eval import (
    aggregate_matrix_status,
    check_statuses,
    domain_status,
)
from audit_code_contract_types import Check, Evidence, OFFICIAL_REFS


_MATRIX_PATH = rel(Path(__file__))


def build_contract_matrix_gate(checks: list[Check]) -> Check:
    statuses = check_statuses(checks)
    evidence_rows: list[Evidence] = []
    domain_statuses: list[str] = []
    for spec in CONTRACT_DOMAINS:
        required = tuple(spec["required"])
        status, issues = domain_status(statuses, required, spec["warn_ok"])
        domain = str(spec["domain"])
        domain_statuses.append(status)
        suffix = "ok" if not issues else "; ".join(issues)
        evidence_rows.append(
            Evidence(
                path=_MATRIX_PATH,
                line=None,
                snippet=f"{domain}: {status} ({suffix})",
            )
        )

    status = aggregate_matrix_status(domain_statuses)
    return Check(
        check_id="source_contract_matrix_gate",
        status=status,
        title="Contract matrix covers time, sensor I/O, RC I/O, thruster, plant input, and dynamic fluidcoef",
        conclusion=(
            "The source audit includes every required contract axis before controller parity or plant replay tuning. "
            "WARN means a known firmware or replay-gate limitation is documented; FAIL means a contract axis is "
            "missing or broken."
        ),
        evidence=evidence_rows,
        official_refs=[
            OFFICIAL_REFS["ardupilot_json_sitl"],
            OFFICIAL_REFS["mavlink_rc_channels_override"],
            OFFICIAL_REFS["mavlink_servo_output_raw"],
            OFFICIAL_REFS["mujoco_fluid"],
            OFFICIAL_REFS["bar30_pressure_sensor"],
        ],
    )


__all__ = ["CONTRACT_DOMAINS", "build_contract_matrix_gate"]
