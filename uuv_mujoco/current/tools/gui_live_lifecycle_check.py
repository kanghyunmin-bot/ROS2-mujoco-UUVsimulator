#!/usr/bin/env python3
"""Live GUI simulator-stack lifecycle check.

This intentionally calls the same GUI start/reset helpers used by the Tk
buttons, but without requiring a human to click the native window.  It starts
the Docker SITL/MuJoCo stack, runs the axis RC override live check, and then
uses the GUI reset worker path to clean up.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.config_paths import SIM_STACK_DIR  # noqa: E402
from gui.process_scan import matching_process_commands  # noqa: E402
from gui.process_termination import terminate_process_group  # noqa: E402
from gui.sim_stack_env_contract import build_gui_sim_stack_env  # noqa: E402
from gui.sim_stack_extra_args import normalize_sim_extra_args  # noqa: E402
from gui.sim_stack_initial_depth_args import build_initial_depth_args  # noqa: E402
from gui.sim_stack_process_probe import external_sim_stack_commands, wait_for_external_sim_stack_exit  # noqa: E402
from gui.sim_stack_reset_commands import reset_sim_stack_blocking, stop_docker_sitl_blocking  # noqa: E402
from gui.sim_stack_reset_worker import run_sim_stack_reset  # noqa: E402
from gui.sim_stack_start_runtime import _start_sim_stack  # noqa: E402


class _BoolVar:
    def __init__(self, value: bool = False) -> None:
        self.value = bool(value)

    def get(self) -> bool:
        return bool(self.value)

    def set(self, value: bool) -> None:
        self.value = bool(value)


class _Node:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.rc_release_count = 0

    def push_event(self, event: str) -> None:
        self.events.append(event)

    def publish_rc_release(self) -> None:
        self.rc_release_count += 1
        self.events.append("publish_rc_release")


@dataclass
class _LiveGuiOwner:
    backend: str
    env_overrides: dict[str, str]
    statuses: list[str] = field(default_factory=list)
    events: list[str] = field(default_factory=list)
    _sim_stack_process: Any | None = None
    _sim_stack_log_path: Path | None = None
    _sim_stack_thread: Any | None = None
    _sim_stack_owned_by_gui: bool = False
    _external_sim_stack_running_cached: bool = False

    def __post_init__(self) -> None:
        self.node = _Node(self.events)
        self.rc_override_enabled = _BoolVar(False)

    def _tracked_sim_stack_running(self) -> bool:
        return self._sim_stack_process is not None and self._sim_stack_process.poll() is None

    def _external_sim_stack_running(self) -> bool:
        return bool(external_sim_stack_commands(matching_process_commands))

    def _sim_stack_backend(self) -> str:
        return self.backend

    def _gui_sim_stack_env(self) -> dict[str, str]:
        base_env = dict(os.environ)
        base_env.update(self.env_overrides)
        return build_gui_sim_stack_env(base_env, backend=self.backend, sim_stack_dir=SIM_STACK_DIR)

    def _rc_replay_running(self) -> bool:
        return False

    def _stop_rc_replay(self) -> None:
        self.events.append("stop_rc_replay")

    def _set_sim_stack_status(self, text: str) -> None:
        self.statuses.append(str(text))

    def _refresh_sim_stack_controls(self) -> None:
        self.events.append("refresh_sim_stack_controls")

    def _watch_sim_stack_output(self, proc: Any, log_path: Path) -> None:
        self.events.append(f"watch_log:{log_path}")
        try:
            proc.wait()
        except Exception as exc:
            self.events.append(f"watch_error:{exc}")

    def _gui_external_mavros_controls_enabled(self) -> bool:
        return False

    def _env_flag(self, name: str, default: bool = False) -> bool:
        raw = os.environ.get(name)
        if raw is None:
            return bool(default)
        return raw.strip().lower() in {"1", "true", "yes", "on"}

    def _arg_present(self, args: list[str], option: str) -> bool:
        return option in args

    def _append_mavros_surface_args(self, cmd: list[str]) -> None:
        if self._gui_external_mavros_controls_enabled():
            cmd.append("--ros2-real-pkg-compat")
            self.node.push_event("MAVROS surface: external node owns arm/mode/RC")
            return
        self.node.push_event("MAVROS surface: internal sim bridge owns arm/mode/RC")

    def _normalized_sim_extra_args(self, extra_args: list[str] | None) -> list[str]:
        result = normalize_sim_extra_args(extra_args, os.environ, platform_name=sys.platform)
        for event in result.events:
            self.node.push_event(event)
        return list(result.args)

    def _append_initial_depth_args(self, cmd: list[str], launch_extra_args: list[str]) -> None:
        result = build_initial_depth_args(os.environ, launch_extra_args=launch_extra_args)
        cmd.extend(result.args)
        for event in result.events:
            self.node.push_event(event)

    def _wait_for_external_sim_stack_exit(self, timeout_s: float = 10.0) -> bool:
        exited, still_running = wait_for_external_sim_stack_exit(
            matching_process_commands,
            timeout_s=timeout_s,
        )
        self._external_sim_stack_running_cached = still_running
        return exited

    def _stop_docker_sitl_blocking(self, timeout_s: float = 12.0) -> None:
        stop_docker_sitl_blocking(timeout_s=timeout_s)

    def _reset_sim_stack_blocking(self, timeout_s: float = 12.0) -> None:
        reset_sim_stack_blocking(timeout_s=timeout_s)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("docker", "native"), default="docker")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "logs" / f"gui_live_lifecycle_{time.strftime('%Y%m%d_%H%M%S')}")
    parser.add_argument("--startup-timeout-s", type=float, default=90.0)
    parser.add_argument("--axis-timeout-s", type=float, default=90.0)
    parser.add_argument("--ekf-contract", default="", help="Optional UUV_EKF_CONTRACT override; empty uses GUI default.")
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--axes", nargs="+", default=["yaw", "heave", "forward", "lateral"])
    return parser.parse_args()


def wait_for_log_pattern(log_path: Path, pattern: str, *, timeout_s: float) -> None:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while time.monotonic() < deadline:
        if log_path.exists() and pattern in log_path.read_text(encoding="utf-8", errors="replace"):
            return
        time.sleep(0.5)
    raise TimeoutError(f"timed out waiting for {pattern!r} in {log_path}")


def axis_check_command(out_dir: Path, axes: list[str]) -> str:
    axes_args = " ".join(axes)
    return f"""set -euo pipefail
source ./.uuv_mujoco_env.sh
source_ros_setup_safely() {{ local setup_file="$1"; set +u; source "$setup_file"; set -u; }}
for candidate in "${{ROS_ENV_SETUP:-}}" "/opt/ros/${{ROS_DISTRO:-humble}}/setup.bash" "$HOME/miniconda3/envs/ros2_mavros/setup.bash" "$HOME/miniconda3/envs/ros2_h311/setup.bash" "$HOME/miniconda3/envs/ros2/setup.bash" "${{CONDA_PREFIX:-}}/setup.bash"; do
  [[ -n "$candidate" && -f "$candidate" ]] || continue
  source_ros_setup_safely "$candidate"
  break
done
if [[ -n "${{ROS_INSTALL_SETUP:-}}" && -f "${{ROS_INSTALL_SETUP}}" ]]; then
  source_ros_setup_safely "$ROS_INSTALL_SETUP"
elif [[ -f "${{ROS_WORKSPACE_DIR:-$PWD/rospkg}}/install/setup.bash" ]]; then
  source_ros_setup_safely "${{ROS_WORKSPACE_DIR:-$PWD/rospkg}}/install/setup.bash"
fi
if [[ -z "${{RMW_IMPLEMENTATION:-}}" ]]; then
  if [[ -f "/opt/ros/${{ROS_DISTRO:-humble}}/lib/librmw_fastrtps_cpp.so" ]]; then
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  elif [[ -f "/opt/ros/${{ROS_DISTRO:-humble}}/lib/librmw_cyclonedds_cpp.so" ]]; then
    export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  else
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  fi
fi
export ROS_LOCALHOST_ONLY="${{ROS_LOCALHOST_ONLY:-0}}" ROS_DISABLE_DAEMON="${{ROS_DISABLE_DAEMON:-1}}"
AXIS_RC_MODE="${{UUV_AXIS_RC_MODE:-ALT_HOLD}}"
AXIS_RC_INPUT_MODE="${{UUV_AXIS_RC_INPUT_MODE:-rc-override}}"
AXIS_RC_PUBLISH_HZ="${{UUV_AXIS_RC_PUBLISH_HZ:-100}}"
AXIS_RC_SAMPLE_HZ="${{UUV_AXIS_RC_SAMPLE_HZ:-30}}"
AXIS_RC_BASELINE_S="${{UUV_AXIS_RC_BASELINE_S:-1.0}}"
AXIS_RC_AXIS_S="${{UUV_AXIS_RC_AXIS_S:-1.8}}"
AXIS_RC_NEUTRAL_S="${{UUV_AXIS_RC_NEUTRAL_S:-1.2}}"
python3 uuv_mujoco/v2.2/tools/axis_rc_override_check.py \\
  --mode "$AXIS_RC_MODE" --input-mode "$AXIS_RC_INPUT_MODE" \\
  --axes {axes_args} \\
  --command 0.5 \\
  --baseline-s "$AXIS_RC_BASELINE_S" --axis-s "$AXIS_RC_AXIS_S" --neutral-s "$AXIS_RC_NEUTRAL_S" \\
  --sample-hz "$AXIS_RC_SAMPLE_HZ" --publish-hz "$AXIS_RC_PUBLISH_HZ" \\
  --out-dir {out_dir} \\
  --disarm-at-end
"""


def run_axis_check(out_dir: Path, axes: list[str], *, timeout_s: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", "-lc", axis_check_command(out_dir, axes)],
        cwd=str(REPO_ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_s,
        check=False,
    )


def cleanup(owner: _LiveGuiOwner) -> None:
    try:
        if owner._tracked_sim_stack_running():
            terminate_process_group(owner._sim_stack_process, timeout_s=5.0)
    finally:
        run_sim_stack_reset(owner)


def write_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    env_overrides = {
        "UUV_MUJOCO_SKIP_FRESHNESS_CHECK": "1",
        "ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK": "0",
        "ROS2_UUV_MAVROS_RC_PWM_SPAN": "400",
        "UUV_GUI_RC_PWM_SPAN": "400",
        "UUV_GUI_PILOT_CONTROL_MODE": "rc_override",
        "UUV_RUN_MODE": "closed_loop",
        "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE": "0",
        "UUV_REAL_START_STATE": "0",
    }
    if args.ekf_contract:
        env_overrides["UUV_EKF_CONTRACT"] = args.ekf_contract

    owner = _LiveGuiOwner(backend=args.backend, env_overrides=env_overrides)
    axis_out = args.out_dir / "axis_rc"
    summary_path = args.out_dir / "gui_live_lifecycle_summary.json"
    payload: dict[str, Any] = {
        "out_dir": str(args.out_dir),
        "backend": args.backend,
        "env_overrides": env_overrides,
        "started": False,
        "startup_ready": False,
        "axis_check_returncode": None,
        "cleanup_attempted": False,
    }
    try:
        extra_args = ["--headless"] if args.headless else []
        _start_sim_stack(owner, extra_args=extra_args)
        payload["statuses_after_start"] = list(owner.statuses)
        payload["events_after_start"] = list(owner.events)
        payload["started"] = bool(owner._tracked_sim_stack_running() and owner._sim_stack_owned_by_gui)
        if not payload["started"]:
            raise RuntimeError("GUI start helper did not start a GUI-owned stack")
        if owner._sim_stack_log_path is None:
            raise RuntimeError("GUI start helper did not record a log path")
        payload["log_path"] = str(owner._sim_stack_log_path)
        wait_for_log_pattern(owner._sim_stack_log_path, "startup handoff", timeout_s=args.startup_timeout_s)
        payload["startup_ready"] = True
        axis_result = run_axis_check(axis_out, list(args.axes), timeout_s=args.axis_timeout_s)
        payload["axis_check_returncode"] = axis_result.returncode
        payload["axis_check_stdout"] = axis_result.stdout
        payload["axis_check_stderr"] = axis_result.stderr
        payload["axis_out_dir"] = str(axis_out)
        if axis_result.returncode != 0:
            raise RuntimeError(f"axis RC check failed with rc={axis_result.returncode}")
        return 0
    except Exception as exc:
        payload["error"] = str(exc)
        return 1
    finally:
        payload["cleanup_attempted"] = True
        try:
            cleanup(owner)
            payload["cleanup_statuses"] = list(owner.statuses)
            payload["cleanup_events"] = list(owner.events)
            payload["external_stack_after_cleanup"] = external_sim_stack_commands(matching_process_commands)
        except Exception as exc:
            payload["cleanup_error"] = str(exc)
        write_summary(summary_path, payload)
        print(f"wrote {summary_path}")
        if payload.get("axis_out_dir"):
            print(f"axis_out={payload['axis_out_dir']}")


if __name__ == "__main__":
    raise SystemExit(main())
