"""Python and mjpython executable candidate discovery for dev OS checks."""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

from dev_os_compat_common import HOME
from dev_os_compat_exec import executable_path


def runtime_python_candidate_inputs(explicit_python: str | None = None) -> tuple[str | Path | None, ...]:
    return (
        explicit_python,
        os.environ.get("MJ311_PYTHON"),
        Path(os.environ["MJ311_ROOT"]) / "bin" / "python" if os.environ.get("MJ311_ROOT") else None,
        HOME / ".venvs" / "uuv_mujoco" / "bin" / "python",
        HOME / ".venvs" / "mujoco311" / "bin" / "python",
        HOME / "miniconda3" / "envs" / "ros2_h311" / "bin" / "python",
        sys.executable,
        shutil.which("python3"),
        shutil.which("python"),
    )


def runtime_mjpython_candidate_inputs(explicit_mjpython: str | None = None) -> tuple[str | Path | None, ...]:
    return (
        explicit_mjpython,
        os.environ.get("MJ311_MJPYTHON"),
        Path(os.environ["MJ311_ROOT"]) / "bin" / "mjpython" if os.environ.get("MJ311_ROOT") else None,
        HOME / ".venvs" / "uuv_mujoco" / "bin" / "mjpython",
        HOME / ".venvs" / "mujoco311" / "bin" / "mjpython",
        HOME / "miniconda3" / "envs" / "ros2_h311" / "bin" / "mjpython",
        shutil.which("mjpython"),
    )


def unique_executable_candidates(raw_candidates: tuple[str | Path | None, ...]) -> list[Path]:
    candidates: list[Path] = []
    for raw in raw_candidates:
        path = executable_path(raw)
        if path is not None and path not in candidates:
            candidates.append(path)
    return candidates


def runtime_python_candidates(explicit_python: str | None = None) -> list[Path]:
    return unique_executable_candidates(runtime_python_candidate_inputs(explicit_python))


def runtime_mjpython_candidates(explicit_mjpython: str | None = None) -> list[Path]:
    candidates = unique_executable_candidates(runtime_mjpython_candidate_inputs(explicit_mjpython))
    for python_bin in runtime_python_candidates():
        path = executable_path(python_bin.parent / "mjpython")
        if path is not None and path not in candidates:
            candidates.append(path)
    return candidates


__all__ = [
    "runtime_mjpython_candidate_inputs",
    "runtime_mjpython_candidates",
    "runtime_python_candidate_inputs",
    "runtime_python_candidates",
    "unique_executable_candidates",
]
