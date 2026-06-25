"""Grouped check execution for development OS compatibility."""

from __future__ import annotations

import platform

from dev_os_compat_common import CheckResult, normalize_target_os
from dev_os_compat_runtime import (
    check_display,
    check_mjpython,
    check_mujoco,
    check_python,
    select_runtime_python,
)
from dev_os_compat_system import (
    check_docker,
    check_docker_host_contract,
    check_ros2_env,
    check_sitl_paths,
    check_ubuntu_migration_contract,
)


def add_host_target_results(results: list[CheckResult], *, target_os: str) -> str:
    target_system = normalize_target_os(target_os)
    results.append(CheckResult("host_os", "pass", f"{platform.system()} {platform.release()} ({platform.machine()})"))
    results.append(CheckResult("target_os", "pass", target_system))
    return target_system


def add_runtime_results(results: list[CheckResult], *, args) -> dict | None:
    python_payload = select_runtime_python(
        results,
        explicit_python=args.python,
        require_viewer=bool(args.require_viewer),
    )
    check_python(results, python_payload)
    check_mujoco(results, python_payload, require_viewer=bool(args.require_viewer))
    check_mjpython(
        results,
        headless=bool(args.headless),
        require_viewer=bool(args.require_viewer),
        explicit_mjpython=args.mjpython,
        runtime_viewer_ok=bool(python_payload and python_payload.get("viewer_ok")),
    )
    return python_payload


def add_system_results(results: list[CheckResult], *, args, target_system: str) -> None:
    check_display(results, headless=bool(args.headless), target_system=target_system)
    check_docker(results)
    check_docker_host_contract(results, target_system=target_system)
    check_ubuntu_migration_contract(results, target_system=target_system)
    check_sitl_paths(results)
    check_ros2_env(results)


__all__ = ["add_host_target_results", "add_runtime_results", "add_system_results"]
