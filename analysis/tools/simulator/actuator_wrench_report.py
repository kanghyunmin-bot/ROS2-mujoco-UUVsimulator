"""Report writers for actuator wrench audits."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from actuator_wrench_common import AXES


def fmt(value: float) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:+.6f}"


def write_markdown(path: Path, result: dict[str, Any]) -> None:
    lines = [
        "# Actuator Wrench Contract Audit",
        "",
        f"- scene: `{result['scene']}`",
        f"- profile: `{result['profile_name']}`",
        f"- thruster_params: `{result['thruster_params']}`",
        f"- unit_gains: `{result['unit_gains']}`",
        "",
        "All wrench components are body-frame FRD. Values are normalized per unit",
        "axis command after ArduSub motor direction and simulator servo sign mapping.",
        "",
        "| axis | Fx surge | Fy right | Fz down | Tx roll | Ty pitch | Tz yaw | force leak | torque leak |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for axis in AXES:
        item = result["axes"][axis]["summary"]
        lines.append(
            "| "
            + " | ".join(
                [
                    axis,
                    fmt(item["surge_force"]),
                    fmt(item["right_force"]),
                    fmt(item["down_force"]),
                    fmt(item["roll_torque"]),
                    fmt(item["pitch_torque"]),
                    fmt(item["yaw_torque"]),
                    fmt(item.get("force_offaxis_ratio", math.nan)),
                    fmt(item.get("torque_offaxis_ratio", math.nan)),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "## Interpretation Hints",
            "",
            "- Roll/pitch/yaw axes should primarily create torque with minimal translational force leakage.",
            "- Forward/lateral/heave axes should primarily create force with minimal off-axis force leakage.",
            "- Non-zero torque on force axes can be physical if the real vehicle has the same lever-arm asymmetry; otherwise it is a contract mismatch candidate.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


__all__ = ["fmt", "write_markdown"]
