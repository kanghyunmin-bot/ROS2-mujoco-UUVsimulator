#!/usr/bin/env python3
"""Run the fast non-live contract sanity gate for the v2.2 runtime.

This gate intentionally avoids launching the interactive MuJoCo viewer, ROS 2,
or ArduSub. It is the quick handoff check for refactors: syntax,
static source contracts, GUI readiness policy, command-link policy, RC frame
semantics, initial hold, sensor start-state helpers, low-load in-process MuJoCo
plant checks, hydrostatic helpers, and thruster parameter loading.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parents[1]
TOOLS = ROOT / "tools"
DEFAULT_OUT = REPO_ROOT / "analysis" / "simulator_research" / "00_current_contract" / "fast_contract_sanity"


@dataclass(frozen=True)
class StepResult:
    name: str
    status: str
    returncode: int
    duration_s: float
    command: list[str]
    stdout_tail: str
    stderr_tail: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument(
        "--include-host",
        action="store_true",
        help="Also run host OS compatibility checks; warnings may reflect local shell state.",
    )
    return parser.parse_args()


def _tail(text: str, *, limit: int = 6000) -> str:
    if len(text) <= limit:
        return text
    return text[-limit:]


def _runtime_cache_dir(name: str) -> Path:
    roots = [
        Path(os.environ["TMPDIR"]) if os.environ.get("TMPDIR") else None,
        Path("/tmp"),
        Path.home() / ".cache" / "uuv_mujoco",
    ]
    for root in roots:
        if root is None:
            continue
        try:
            root.mkdir(parents=True, exist_ok=True)
            path = root / name
            path.mkdir(parents=True, exist_ok=True)
            return path
        except OSError:
            continue
    fallback = ROOT / ".cache" / name
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def _env() -> dict[str, str]:
    env = dict(os.environ)
    env.setdefault("PYTHONPYCACHEPREFIX", str(_runtime_cache_dir("pycache")))
    env.setdefault("MPLCONFIGDIR", str(_runtime_cache_dir("matplotlib")))
    return env


def run_step(name: str, command: list[str], *, timeout_s: float) -> StepResult:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env=_env(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
            check=False,
        )
        returncode = int(completed.returncode)
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as exc:
        returncode = 124
        stdout = exc.stdout or ""
        stderr = (exc.stderr or "") + f"\nTIMEOUT after {timeout_s:.1f}s"
    duration_s = time.monotonic() - started
    return StepResult(
        name=name,
        status="PASS" if returncode == 0 else "FAIL",
        returncode=returncode,
        duration_s=round(duration_s, 3),
        command=command,
        stdout_tail=_tail(str(stdout)),
        stderr_tail=_tail(str(stderr)),
    )


def _python_script(script: str, *args: str) -> list[str]:
    return [sys.executable, str(TOOLS / script), *args]


def _runtime_python_script(script: str, *args: str) -> list[str]:
    runtime_python = (
        os.environ.get("MJ311_PYTHON")
        or "/Users/kanghyunmin/.venvs/mujoco311/bin/python"
        or sys.executable
    )
    if not Path(runtime_python).exists():
        runtime_python = sys.executable
    return [runtime_python, str(TOOLS / script), *args]


def build_steps(args: argparse.Namespace) -> list[tuple[str, list[str], float]]:
    compile_targets = [
        str(ROOT / "tools"),
        str(ROOT / "bridge"),
        str(ROOT / "sim"),
        str(ROOT / "physics"),
        str(ROOT / "gui"),
    ]
    steps: list[tuple[str, list[str], float]] = [
        ("compileall_core", [sys.executable, "-m", "compileall", "-q", *compile_targets], args.timeout_s),
        ("rc_frame_contract", _python_script("check_rc_frame_contract.py"), args.timeout_s),
        ("sitl_frame_contract", _python_script("check_sitl_frame_contract.py"), args.timeout_s),
        ("ros2_command_payload", _python_script("check_ros2_command_payload.py"), args.timeout_s),
        ("bridge_live_imports", _runtime_python_script("check_bridge_live_imports.py"), args.timeout_s),
        ("json_servo_receiver", _python_script("check_json_servo_receiver.py"), args.timeout_s),
        ("sitl_servo_runtime", _python_script("check_sitl_servo_runtime.py"), args.timeout_s),
        ("immediate_sensor_replay_reply", _python_script("check_immediate_sensor_replay_reply.py"), args.timeout_s),
        ("mujoco_velocity_contract", _runtime_python_script("check_mujoco_velocity_contract.py"), args.timeout_s),
        ("sim_clock_contract", _python_script("check_sim_clock_contract.py"), args.timeout_s),
        ("sim_runtime_smooth_contract", _python_script("check_sim_runtime_smooth_contract.py"), args.timeout_s),
        ("sitl_command_link_readiness", _python_script("check_sitl_command_link_readiness.py"), args.timeout_s),
        ("command_latency_contract", _python_script("check_command_latency_contract.py"), args.timeout_s),
        ("sitl_external_nav_contract", _python_script("check_sitl_external_nav_contract.py"), args.timeout_s),
        ("mavlink_message_interval", _python_script("check_mavlink_message_interval.py"), args.timeout_s),
        ("runtime_readiness_policy", _python_script("check_runtime_readiness_policy.py"), args.timeout_s),
        ("gui_backend_selection", _python_script("check_gui_backend_selection.py"), args.timeout_s),
        ("gui_entry_contract", _python_script("check_gui_entry_contract.py"), args.timeout_s),
        ("gui_start_contract", _python_script("check_gui_start_contract.py"), args.timeout_s),
        ("gui_readiness_contract", _python_script("check_gui_readiness_contract.py"), args.timeout_s),
        ("gui_file_persistence", _python_script("check_gui_file_persistence.py"), args.timeout_s),
        ("process_log_lifecycle", _python_script("check_process_log_lifecycle.py"), args.timeout_s),
        (
            "async_camera_snapshot_reuse",
            _runtime_python_script("check_async_camera_snapshot_reuse.py"),
            args.timeout_s,
        ),
        ("web_gui_contract", _python_script("check_web_gui_contract.py"), args.timeout_s),
        ("gui_rc_fast_loop_contract", _python_script("check_gui_rc_fast_loop_contract.py"), args.timeout_s),
        ("gui_rc_replay_decode", _python_script("check_gui_rc_replay_decode.py"), args.timeout_s),
        ("gui_buoy_layout_editor", _python_script("check_gui_buoy_layout_editor.py"), args.timeout_s),
        ("competition_course_scene", _python_script("check_competition_course_scene.py"), args.timeout_s),
        ("sim_exit_process_cleanup", _python_script("test_sim_exit_process_cleanup.py"), args.timeout_s),
        ("gui_depth_freshness", _python_script("test_gui_depth_freshness.py"), args.timeout_s),
        (
            "web_gui_runtime_contracts",
            _python_script("test_web_gui_runtime_contracts.py"),
            args.timeout_s,
        ),
        ("rc3_neutral_contract", _python_script("test_rc3_neutral_contract.py"), args.timeout_s),
        (
            "althold_throttle_normalization",
            _python_script("test_althold_throttle_normalization.py"),
            args.timeout_s,
        ),
        ("gui_ros_python_contract", _python_script("check_gui_ros_python_contract.py"), args.timeout_s),
        ("gui_arm_mode_command_contract", _python_script("check_gui_arm_mode_command_contract.py"), args.timeout_s),
        ("gui_initial_depth_contract", _python_script("check_gui_initial_depth_contract.py"), args.timeout_s),
        (
            "initial_depth_auto_release_contract",
            _python_script("check_initial_depth_auto_release_contract.py"),
            args.timeout_s,
        ),
        ("gui_pilot_toggle_contract", _python_script("check_gui_pilot_toggle_contract.py"), args.timeout_s),
        ("gui_pilot_auto_enable_contract", _python_script("check_gui_pilot_auto_enable_contract.py"), args.timeout_s),
        ("dist_rc_override_path", _python_script("check_dist_rc_override_path.py"), args.timeout_s),
        ("manual_control_input_scaling", _python_script("check_manual_control_input_scaling.py"), args.timeout_s),
        ("axis_rc_health_contract", _python_script("check_axis_rc_health_contract.py"), args.timeout_s),
        ("axis_rc_latency_metrics", _python_script("check_axis_rc_latency_metrics.py"), args.timeout_s),
        ("mavros_rcout_publish_policy", _python_script("check_mavros_rcout_publish_policy.py"), args.timeout_s),
        ("ros2_replay_rcout", _python_script("check_ros2_replay_rcout.py"), args.timeout_s),
        ("initial_hold_pose", _python_script("check_initial_hold_pose.py"), args.timeout_s),
        ("real_start_measurements", _python_script("check_real_start_measurements.py"), args.timeout_s),
        ("ros2_dvl_messages", _python_script("check_ros2_dvl_messages.py"), args.timeout_s),
        ("strict_real_pkg_surface", _python_script("check_strict_real_pkg_surface.py"), args.timeout_s),
        ("static_context_publisher", _python_script("check_static_context_publisher.py"), args.timeout_s),
        ("physics_contract_geometry", _python_script("check_physics_contract_geometry.py"), args.timeout_s),
        (
            "physics_contract_neutral_metrics",
            _python_script("check_physics_contract_neutral_metrics.py"),
            args.timeout_s,
        ),
        ("hydrostatic_buoyancy_points", _python_script("check_hydrostatic_buoyancy_points.py"), args.timeout_s),
        ("model_binary_cache", _runtime_python_script("check_model_binary_cache.py"), args.timeout_s),
        ("model_runtime_setup", _runtime_python_script("check_model_runtime_setup.py"), args.timeout_s),
        (
            "course_buoy_contact_snapshot",
            _python_script("check_course_buoy_contact_snapshot.py"),
            args.timeout_s,
        ),
        (
            "viewer_pause_publish_contract",
            _python_script("check_viewer_pause_publish_contract.py"),
            args.timeout_s,
        ),
        ("physics_runtime_hydrostatic", _python_script("check_physics_runtime_hydrostatic.py"), args.timeout_s),
        ("sim_profile_safety_contract", _python_script("check_sim_profile_safety_contract.py"), args.timeout_s),
        ("underwater_flow_contract", _python_script("check_underwater_flow_contract.py"), args.timeout_s),
        ("vehicle_fluid_free_decay", _runtime_python_script("check_vehicle_fluid_free_decay.py"), args.timeout_s),
        ("thruster_param_loader", _python_script("check_thruster_param_loader.py"), args.timeout_s),
        ("thruster_performance_curves", _python_script("check_thruster_performance_curves.py"), args.timeout_s),
        ("odometry_publish_builders", _python_script("check_odometry_publish_builders.py"), args.timeout_s),
        ("ros2_ping360_messages", _python_script("check_ros2_ping360_messages.py"), args.timeout_s),
        ("hydrophone_audio_timing", _python_script("check_hydrophone_audio_timing.py"), args.timeout_s),
        ("single_hydrophone_homing_math", _python_script("check_single_hydrophone_homing_math.py"), args.timeout_s),
        ("homing_direction_viewer", _python_script("check_homing_direction_viewer.py"), args.timeout_s),
        ("vision_buoy_pipeline", _python_script("check_vision_buoy_pipeline.py"), args.timeout_s),
        ("yolo_buoy_overlay_contract", _python_script("check_yolo_buoy_overlay_contract.py"), args.timeout_s),
        ("rviz_mission_visualizer_contract", _python_script("check_rviz_mission_visualizer_contract.py"), args.timeout_s),
        ("ros2_sitl_command_override", _python_script("check_ros2_sitl_command_override.py"), args.timeout_s),
        ("ping360_stl_io", _python_script("check_filter_ping360_stl_io.py"), args.timeout_s),
        (
            "source_contract_audit",
            _python_script(
                "audit_code_contract_sources.py",
                "--out-dir",
                str(args.out_dir / "source_contract_audit"),
            ),
            args.timeout_s,
        ),
    ]
    if args.include_host:
        steps.append(
            (
                "dev_os_compat_headless",
                _python_script("check_dev_os_compat.py", "--headless", "--json"),
                args.timeout_s,
            )
        )
    return steps


def write_outputs(out_dir: Path, results: list[StepResult]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "generated_at_unix": time.time(),
        "root": str(ROOT),
        "counts": {
            "pass": sum(1 for result in results if result.status == "PASS"),
            "fail": sum(1 for result in results if result.status == "FAIL"),
        },
        "results": [asdict(result) for result in results],
    }
    (out_dir / "fast_contract_sanity.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    lines = [
        "# Fast Contract Sanity",
        "",
        f"- root: `{ROOT}`",
        f"- pass: {summary['counts']['pass']}",
        f"- fail: {summary['counts']['fail']}",
        "",
        "| Step | Status | Seconds |",
        "| --- | --- | ---: |",
    ]
    for result in results:
        lines.append(f"| `{result.name}` | {result.status} | {result.duration_s:.3f} |")
    lines.append("")
    lines.append("Full stdout/stderr tails are in `fast_contract_sanity.json`.")
    (out_dir / "fast_contract_sanity.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    results = [run_step(name, command, timeout_s=timeout_s) for name, command, timeout_s in build_steps(args)]
    write_outputs(args.out_dir, results)
    for result in results:
        print(f"{result.status:4s} {result.name} ({result.duration_s:.3f}s)")
    failed = [result.name for result in results if result.status == "FAIL"]
    if failed:
        print("failed=" + ",".join(failed))
        return 1
    print(f"wrote {args.out_dir / 'fast_contract_sanity.json'}")
    print(f"wrote {args.out_dir / 'fast_contract_sanity.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
