"""Python probe evaluation helpers for dev OS compatibility checks."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from dev_os_compat_python_probe import probe_python


@dataclass(frozen=True)
class RuntimePythonProbe:
    candidate: Path
    status: int
    payload: dict[str, object] | None
    output: str
    summary: str


def runtime_python_payload_passes(payload: dict[str, object], *, require_viewer: bool) -> bool:
    version = payload.get("version", [0, 0, 0])
    version_ok = list(version) >= [3, 10, 0]
    mujoco_ok = bool(payload.get("mujoco_ok"))
    viewer_ok = (not require_viewer) or bool(payload.get("viewer_ok"))
    return version_ok and mujoco_ok and viewer_ok


def runtime_python_probe_summary(
    candidate: Path,
    payload: dict[str, object] | None,
    output: str,
    *,
    require_viewer: bool,
) -> str:
    if payload is None:
        return f"{candidate}: probe failed: {output}"

    version = payload.get("version", [0, 0, 0])
    mujoco_ok = bool(payload.get("mujoco_ok"))
    viewer_ok = (not require_viewer) or bool(payload.get("viewer_ok"))
    return (
        f"{candidate}: py={'.'.join(map(str, version))} "
        f"mujoco={payload.get('mujoco_version') if mujoco_ok else payload.get('mujoco_error')} "
        f"viewer={'ok' if viewer_ok else payload.get('viewer_error', 'not checked')}"
    )


def probe_runtime_python(candidate: Path, *, require_viewer: bool) -> RuntimePythonProbe:
    status, payload, output = probe_python(candidate, require_viewer=require_viewer)
    return RuntimePythonProbe(
        candidate=candidate,
        status=status,
        payload=payload,
        output=output,
        summary=runtime_python_probe_summary(
            candidate,
            payload,
            output,
            require_viewer=require_viewer,
        ),
    )


__all__ = [
    "RuntimePythonProbe",
    "probe_runtime_python",
    "runtime_python_payload_passes",
    "runtime_python_probe_summary",
]
