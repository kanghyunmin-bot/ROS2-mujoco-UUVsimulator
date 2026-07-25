"""SITL parameter-source loading for closed-loop contract audits."""

from __future__ import annotations

from pathlib import Path

from audit_closed_loop_params import parse_param_file, parse_start_sitl_enforced_params


def load_sitl_param_contract(
    workspace: Path,
    sim_dir: Path,
    sitl_log: Path | None = None,
) -> tuple[dict[str, str], list[str]]:
    sitl_param_sources: list[str] = []
    sitl_defaults_path = workspace / "sim" / "ardupilot" / "Tools" / "autotest" / "default_params" / "sub-6dof.parm"
    sitl_params = parse_param_file(sitl_defaults_path)
    if sitl_defaults_path.exists():
        sitl_param_sources.append(str(sitl_defaults_path))

    real_contract_path = sim_dir / "config" / "ardusub_realrobot_contract.param"
    real_contract_params = parse_param_file(real_contract_path)
    if real_contract_params:
        sitl_params.update(real_contract_params)
        sitl_param_sources.append(str(real_contract_path))

    mav_param_path = workspace / "sim" / "ardupilot" / "mav.parm"
    mav_params = parse_param_file(mav_param_path)
    if mav_params:
        sitl_params.update(mav_params)
        sitl_param_sources.append(str(mav_param_path))

    if sitl_log is not None:
        logged_params = parse_start_sitl_enforced_params(sitl_log)
        if logged_params:
            sitl_params.update(logged_params)
            sitl_param_sources.append(str(sitl_log))
    return sitl_params, sitl_param_sources


__all__ = ["load_sitl_param_contract"]
