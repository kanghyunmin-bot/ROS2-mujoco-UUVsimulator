"""Python and MuJoCo result emitters for dev OS compatibility checks."""

from __future__ import annotations

from dev_os_compat_common import CheckResult


def check_python(results: list[CheckResult], payload: dict[str, object] | None) -> None:
    if payload is None:
        results.append(CheckResult("python", "fail", "runtime Python not resolved"))
        return
    version = payload.get("version", [0, 0, 0])
    version_text = ".".join(map(str, version))
    status = "pass" if list(version) >= [3, 10, 0] else "fail"
    results.append(CheckResult("python", status, f"{payload.get('executable')} {version_text}"))


def check_mujoco(results: list[CheckResult], payload: dict[str, object] | None, *, require_viewer: bool) -> None:
    if payload is None:
        results.append(CheckResult("mujoco_import", "fail", "runtime Python not resolved"))
        return
    if payload.get("mujoco_ok"):
        results.append(CheckResult("mujoco_import", "pass", f"mujoco {payload.get('mujoco_version')}"))
    else:
        results.append(CheckResult("mujoco_import", "fail", str(payload.get("mujoco_error", "unknown error"))))
    if require_viewer:
        if payload.get("viewer_ok"):
            results.append(CheckResult("mujoco_viewer_import", "pass", "mujoco.viewer import ok"))
        else:
            results.append(CheckResult("mujoco_viewer_import", "fail", str(payload.get("viewer_error", "unknown error"))))


__all__ = ["check_mujoco", "check_python"]
