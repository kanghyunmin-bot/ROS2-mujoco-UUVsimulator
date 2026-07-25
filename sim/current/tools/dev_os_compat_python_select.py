"""Runtime Python selection policy for dev OS compatibility checks."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from dev_os_compat_python_eval import (
    RuntimePythonProbe,
    probe_runtime_python,
    runtime_python_payload_passes,
)


@dataclass(frozen=True)
class RuntimePythonSelection:
    payload: dict[str, object] | None
    fallback_payload: dict[str, object] | None
    passed_candidate: Path | None
    probes: tuple[RuntimePythonProbe, ...]

    @property
    def passed(self) -> bool:
        return self.payload is not None and self.passed_candidate is not None

    def failure_detail(self) -> str:
        return "no candidate passed; " + " | ".join(probe.summary for probe in self.probes)


def select_first_passing_runtime_python(
    candidates: list[Path],
    *,
    require_viewer: bool,
) -> RuntimePythonSelection:
    probes: list[RuntimePythonProbe] = []
    fallback_payload: dict[str, object] | None = None
    for candidate in candidates:
        probe = probe_runtime_python(candidate, require_viewer=require_viewer)
        probes.append(probe)
        if probe.payload is None:
            continue
        if fallback_payload is None:
            fallback_payload = probe.payload
        if probe.status == 0 and runtime_python_payload_passes(probe.payload, require_viewer=require_viewer):
            return RuntimePythonSelection(
                payload=probe.payload,
                fallback_payload=fallback_payload,
                passed_candidate=candidate,
                probes=tuple(probes),
            )
    return RuntimePythonSelection(
        payload=None,
        fallback_payload=fallback_payload,
        passed_candidate=None,
        probes=tuple(probes),
    )


__all__ = ["RuntimePythonSelection", "select_first_passing_runtime_python"]
