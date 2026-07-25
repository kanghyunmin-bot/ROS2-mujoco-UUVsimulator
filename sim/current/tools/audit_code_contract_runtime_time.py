"""Active-runtime time contract source check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_runtime_time_sources import (
    runtime_time_contract_passes,
    runtime_time_evidence,
    runtime_time_sources,
)
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_runtime_time_contract_check(runtime_paths: dict[str, Path]) -> Check:
    sources = runtime_time_sources(runtime_paths)
    ok = runtime_time_contract_passes(sources)
    return Check(
        check_id="active_runtime_time_contract_sim_publish_wall_transport",
        status="PASS" if ok else "FAIL",
        title="Runtime separates sim-time sensor publishing from wall-time transport polling",
        conclusion=(
            "ROS sensor output is gated by monotonic sim time and sensor_dt; sensor-replay can use a "
            "monotonic JSON-servo frame clock; MAVLink polling, SERVO_OUTPUT_RAW requests, RC keepalive, "
            "and viewer catch-up use wall-clock cadence."
        ),
        evidence=runtime_time_evidence(sources),
        official_refs=[OFFICIAL_REFS["mavlink_servo_output_raw"]],
    )


__all__ = ["build_runtime_time_contract_check"]
