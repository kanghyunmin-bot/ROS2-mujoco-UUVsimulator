"""Ubuntu migration contract checks for development OS compatibility."""

from __future__ import annotations

from dev_os_compat_common import ROOT, CheckResult


def check_ubuntu_migration_contract(results: list[CheckResult], *, target_system: str) -> None:
    if target_system != "Linux":
        results.append(CheckResult("ubuntu_migration_contract", "pass", "not targeting Linux/Ubuntu"))
        return

    launch = ROOT / "launch_uuv_sim.sh"
    docs = ROOT.parents[1] / "documentary" / "simulator" / "contracts" / "DEV_OS_COMPATIBILITY.md"
    workspace = ROOT.parents[1]
    missing = [str(path.relative_to(workspace)) for path in (launch, docs) if not path.exists()]
    if missing:
        results.append(CheckResult("ubuntu_migration_contract", "fail", "missing: " + ", ".join(missing)))
        return

    launch_text = launch.read_text(encoding="utf-8")
    required_markers = {
        "launch_linux_branch": "Linux)" in launch_text,
        "launch_display_contract": "DISPLAY" in launch_text or "WAYLAND_DISPLAY" in launch_text,
        "native_sitl_transport": "SITL_DIRECT_MAVLINK" in launch_text,
    }
    failed = [name for name, ok in required_markers.items() if not ok]
    if failed:
        results.append(CheckResult("ubuntu_migration_contract", "fail", "missing markers: " + ", ".join(failed)))
    else:
        results.append(
            CheckResult(
                "ubuntu_migration_contract",
                "pass",
                "native launcher and documentation are Ubuntu-aware",
            )
        )
