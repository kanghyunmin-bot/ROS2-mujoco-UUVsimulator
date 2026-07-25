"""SITL, active-runtime, and simulator entrypoint path checks."""

from __future__ import annotations

from dev_os_compat_common import ROOT, WORKSPACE, CheckResult
from dev_os_compat_path_checks import append_executable_scripts_check, append_required_files_check
from dev_os_compat_sitl_alias import check_active_runtime_alias


def check_sitl_paths(results: list[CheckResult]) -> None:
    check_active_runtime_alias(results)
    _check_runtime_scripts(results)
    _check_runner_entrypoints(results)
    _check_real_param_contract(results)
    _check_ardupilot_tree(results)


def _check_runtime_scripts(results: list[CheckResult]) -> None:
    scripts = [
        ROOT / "launch_uuv_sim.sh",
        ROOT / "start_sitl_mujoco_mj311.sh",
        ROOT / "reset_uuv_sim.sh",
    ]
    append_executable_scripts_check(
        results,
        name="runtime_scripts",
        paths=scripts,
        pass_detail="launcher scripts present and executable",
    )


def _check_runner_entrypoints(results: list[CheckResult]) -> None:
    runners = [
        ROOT / "run_uuv_mujoco.py",
        ROOT / "run_urdf_full.py",
    ]
    append_required_files_check(
        results,
        name="runtime_runner_entrypoints",
        paths=runners,
        pass_detail="run_uuv_mujoco.py plus run_urdf_full.py compatibility wrapper present",
    )


def _check_real_param_contract(results: list[CheckResult]) -> None:
    real_param = WORKSPACE / "real_robot.param"
    contract_param = ROOT / "config" / "ardusub_realrobot_contract.param"
    if real_param.exists() or contract_param.exists():
        results.append(CheckResult("real_param_contract", "pass", "real/SITL param source found"))
    else:
        results.append(CheckResult("real_param_contract", "fail", "real_robot.param and contract param are missing"))


def _check_ardupilot_tree(results: list[CheckResult]) -> None:
    ardupilot = WORKSPACE / "sim" / "ardupilot"
    results.append(
        CheckResult(
            "ardupilot_tree",
            "pass" if ardupilot.exists() else "warn",
            str(ardupilot) if ardupilot.exists() else "missing; native SITL runtime needs an ArduPilot checkout",
        )
    )
