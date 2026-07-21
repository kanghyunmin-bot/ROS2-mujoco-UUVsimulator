"""Ubuntu migration contract checks for development OS compatibility."""

from __future__ import annotations

from dev_os_compat_common import ROOT, CheckResult


def check_ubuntu_migration_contract(results: list[CheckResult], *, target_system: str) -> None:
    if target_system != "Linux":
        results.append(CheckResult("ubuntu_migration_contract", "pass", "not targeting Linux/Ubuntu"))
        return

    launch = ROOT / "launch_uuv_sim.sh"
    docker_start = ROOT / "start_docker_sitl_mujoco_mj311.sh"
    docs = ROOT / "docs" / "contracts" / "DEV_OS_COMPATIBILITY.md"
    missing = [str(path.relative_to(ROOT)) for path in (launch, docker_start, docs) if not path.exists()]
    if missing:
        results.append(CheckResult("ubuntu_migration_contract", "fail", "missing: " + ", ".join(missing)))
        return

    launch_text = launch.read_text(encoding="utf-8")
    docker_text = docker_start.read_text(encoding="utf-8")
    required_markers = {
        "launch_linux_branch": "Linux)" in launch_text,
        "launch_display_contract": "DISPLAY" in launch_text or "WAYLAND_DISPLAY" in launch_text,
        "docker_host_internal": "host.docker.internal" in docker_text,
        "docker_json_ports": "SITL_JSON_HOST" in docker_text and "SITL_JSON_SERVO_PORT" in docker_text,
    }
    failed = [name for name, ok in required_markers.items() if not ok]
    if failed:
        results.append(CheckResult("ubuntu_migration_contract", "fail", "missing markers: " + ", ".join(failed)))
    else:
        results.append(
            CheckResult(
                "ubuntu_migration_contract",
                "pass",
                "launcher, Docker SITL host routing, JSON ports, and docs are Ubuntu-aware",
            )
        )
