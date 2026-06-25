"""Docker and Docker-host routing checks for development OS compatibility."""

from __future__ import annotations

import shutil

from dev_os_compat_common import WORKSPACE, CheckResult
from dev_os_compat_exec import run_command


def check_docker(results: list[CheckResult]) -> None:
    docker = shutil.which("docker")
    if docker is None:
        results.append(CheckResult("docker_cli", "warn", "docker CLI not found"))
        return
    code, output = run_command([docker, "--version"])
    results.append(CheckResult("docker_cli", "pass" if code == 0 else "warn", output or f"exit={code}"))
    code, output = run_command([docker, "compose", "version"])
    results.append(CheckResult("docker_compose", "pass" if code == 0 else "warn", output or f"exit={code}"))
    code, output = run_command([docker, "ps"])
    status = "pass" if code == 0 else "warn"
    detail = "daemon reachable" if code == 0 else output
    results.append(CheckResult("docker_daemon", status, detail))


def check_docker_host_contract(results: list[CheckResult], *, target_system: str) -> None:
    compose = WORKSPACE / "docker" / "ardusub" / "docker-compose.yml"
    if not compose.exists():
        results.append(CheckResult("docker_host_gateway", "warn", f"compose file not found: {compose}"))
        return
    text = compose.read_text(encoding="utf-8")
    if "host.docker.internal:host-gateway" in text:
        results.append(
            CheckResult(
                "docker_host_gateway",
                "pass",
                "compose maps host.docker.internal to host-gateway for Linux Docker",
            )
        )
    elif target_system == "Linux":
        results.append(
            CheckResult(
                "docker_host_gateway",
                "fail",
                "Linux Docker needs extra_hosts host.docker.internal:host-gateway",
            )
        )
    else:
        results.append(CheckResult("docker_host_gateway", "warn", "no host-gateway mapping in compose"))
