"""Process controls used by the web GUI runtime."""

from __future__ import annotations

import json
import os
import math
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

from .config import (
    APP_ROOT,
    COURSE_LAYOUT_CONFIG_PATH,
    COURSE_SCENE_PATH,
    RC_PWM_SPAN,
    RESET_SIM_STACK_SCRIPT,
    ROS_PACKAGE_DEFAULT_FCU_URL,
    ROS_PACKAGE_DIR,
    ROS_PACKAGE_NAME,
    ROS_PACKAGE_RVIZ_CONFIG,
    ROS_SOURCE_DIR,
    ROS_WORKSPACE_DIR,
    SIM_STACK_DIR,
    TEST_TANK_SCENE_PATH,
)
from .ping360_view_process import build_ping360_view_ros_command
from .pinger_sim_profile import (
    PINGER_HOMING_SIM_PURPOSE,
    normalize_sim_start_purpose,
    pinger_sim_environment,
    pinger_sim_launch_args,
)
from .process_env import arg_present, env_flag, sim_stack_backend
from .process_log_files import IncrementalLogMatcher, allocate_process_log_path
from .process_termination import terminate_exited_process_group, terminate_process_group
from .ros_package_build import ros_package_build_command
from .ros_package_stack import mavros_launch_command
from .ros_tools import prepare_ping360_rviz_config, prepare_ros2_rviz_config, ros_bash_command
from .sim_stack_env import build_gui_sim_stack_env, build_initial_depth_args, normalize_sim_extra_args
from .sim_stack_launch_command import (
    build_sim_stack_launch_command,
    camera_config_from_owner,
    camera_presets_payload,
    normalize_camera_config,
)
from .sim_stack_launch_logs import open_gui_sim_stack_log
from .sim_stack_launch_process import spawn_sim_stack_process
from .sim_stack_launch_target import resolve_sim_stack_launch_target, sim_stack_start_script_error
from .sim_launch_preset import (
    COURSE_CURRENT_PRESET_ID,
    build_sim_launch_preset_args,
    default_sim_launch_preset_id,
    resolve_sim_launch_preset,
    sim_launch_presets_payload,
    validate_sim_launch_preset,
)
from .test_tank_layout_model import (
    COURSE_MODE_TEST_TANK,
    TEST_TANK_HOMING_SUCCESS_RANGE_M,
    TEST_TANK_DEPTH_M,
    load_course_layout_config,
    prepare_active_course_runtime,
)


class WebProcessManager:
    """Owns simulator, ROS, RViz, and Ping360 viewer child processes."""

    def __init__(self, node: Any) -> None:
        self.node = node
        self._lock = threading.Lock()
        self._sim_process: subprocess.Popen[str] | None = None
        self._ros_build_process: subprocess.Popen[str] | None = None
        self._ros_pkg_process: subprocess.Popen[str] | None = None
        self._rviz_process: subprocess.Popen[str] | None = None
        self._ping360_view_process: subprocess.Popen[str] | None = None
        self._pinger_homing_process: subprocess.Popen[str] | None = None
        self._vision_process: subprocess.Popen[str] | None = None
        self._mission_process: subprocess.Popen[str] | None = None
        self._sim_stack_status = "sim: stopped"
        self._ros_pkg_status = "mavros: stopped"
        self._rviz_status = "rviz: stopped"
        self._ping360_view_status = "ping360 view: closed"
        self._pinger_homing_status = "pinger homing: stopped"
        self._vision_status = "vision: stopped"
        self._mission_status = "mission: stopped"
        self._sim_launch_preset_id = default_sim_launch_preset_id()
        self._active_sim_launch_preset_id = ""
        self._active_sim_scene = ""
        self._ros_pkg_fcu_url = ROS_PACKAGE_DEFAULT_FCU_URL
        self._camera_config = camera_config_from_owner(self)
        self._mission_status_path = Path(
            os.environ.get(
                "UUV_GUI_MISSION_STATUS_JSON",
                str(SIM_STACK_DIR / "generated" / "mission_fsm_status.json"),
            )
        )

    def status_payload(self) -> dict[str, Any]:
        sim_stack_status = self._live_sim_stack_status()
        return {
            "sim_running": self._simulation_runtime_available(),
            "ros_build_running": _process_running(self._ros_build_process),
            "mavros_running": _process_running(self._ros_pkg_process),
            "rviz_running": _process_running(self._rviz_process),
            "ping360_view_running": _process_running(self._ping360_view_process),
            "pinger_homing_running": _process_running(self._pinger_homing_process),
            "vision_running": _process_running(self._vision_process),
            "mission_running": _process_running(self._mission_process),
            "sim_stack_status": sim_stack_status,
            "ros_pkg_status": self._ros_pkg_status,
            "rviz_status": self._rviz_status,
            "ping360_view_status": self._ping360_view_status,
            "pinger_homing_status": self._pinger_homing_status,
            "vision_status": self._vision_status,
            "mission_status": self._mission_status,
            "mission_monitor": self.mission_monitor_payload(),
            "ros_pkg_fcu_url": self._ros_pkg_fcu_url,
            "camera_config": self.camera_config_payload(),
            "simulation_config": self.simulation_config_payload(),
        }

    def mission_running(self) -> bool:
        return _process_running(self._mission_process)

    def pinger_homing_running(self) -> bool:
        return _process_running(self._pinger_homing_process)

    def simulation_runtime_available(self) -> bool:
        return self._simulation_runtime_available()

    def stop_all(self) -> None:
        for attr_name, status_attr, label in (
            ("_mission_process", "_mission_status", "mission"),
            ("_pinger_homing_process", "_pinger_homing_status", "pinger homing"),
            ("_vision_process", "_vision_status", "vision"),
            ("_ping360_view_process", "_ping360_view_status", "ping360 view"),
            ("_rviz_process", "_rviz_status", "rviz"),
            ("_ros_pkg_process", "_ros_pkg_status", "mavros"),
            ("_ros_build_process", "_ros_pkg_status", "ros2 build"),
            ("_sim_process", "_sim_stack_status", "sim"),
        ):
            self._terminate_named_process(attr_name, status_attr, label, push_event=False)
        self._clear_mission_status_file()

    def start_sim_stack(
        self,
        *,
        purpose: str | None = None,
        preset_id: str | None = None,
    ) -> dict[str, Any]:
        try:
            start_purpose = normalize_sim_start_purpose(purpose)
            selected_preset = resolve_sim_launch_preset(
                COURSE_CURRENT_PRESET_ID
                if start_purpose == PINGER_HOMING_SIM_PURPOSE
                else preset_id or self._sim_launch_preset_id
            )
            validate_sim_launch_preset(selected_preset)
        except ValueError as exc:
            self._set_sim_stack_status(f"sim start failed: {exc}")
            return {"status": self._sim_stack_status}
        self._clear_stereo_camera_frames()
        self._clear_mission_status_file()
        with self._lock:
            if _process_running(self._sim_process):
                self._sim_stack_status = "sim: already running"
                return {"status": self._sim_stack_status}
        backend = self._sim_stack_backend()
        target = resolve_sim_stack_launch_target(backend)
        error = sim_stack_start_script_error(target.start_script)
        if error:
            self._set_sim_stack_status(error)
            return {"status": self._sim_stack_status}

        course_runtime = None
        if selected_preset.uses_active_course_scene:
            try:
                course_runtime = prepare_active_course_runtime(
                    config_path=COURSE_LAYOUT_CONFIG_PATH,
                    competition_scene_path=COURSE_SCENE_PATH,
                    test_tank_scene_path=TEST_TANK_SCENE_PATH,
                )
            except Exception as exc:
                self._set_sim_stack_status(f"sim start failed: course scene: {exc}")
                return {"status": self._sim_stack_status}
            runtime_mode = course_runtime.mode
            scene_path = course_runtime.scene_path
            pinger_site_name = course_runtime.pinger_site_name
        else:
            runtime_mode = "research_pool"
            scene_path = selected_preset.scene_path
            # The SLAM pool intentionally has no mission pinger target.  Keep
            # the hydrophone surface alive but explicitly report a missing
            # source instead of accidentally indexing a course-scene site.
            pinger_site_name = "research_pool_pinger_unavailable"

        base_env = dict(os.environ)
        if start_purpose == PINGER_HOMING_SIM_PURPOSE:
            # Apply these before building the GUI contract: EKF sensor flags
            # are derived from UUV_EKF_CONTRACT by the environment builder.
            base_env.update(pinger_sim_environment())
        env = build_gui_sim_stack_env(base_env, backend=backend, sim_stack_dir=SIM_STACK_DIR)
        env["ROS2_UUV_HYDROPHONE_PINGER_SITE"] = pinger_site_name
        env["UUV_GUI_COURSE_MODE"] = runtime_mode
        # Keep the bridge's normal acoustic interference profile. Pinger
        # validation must include thruster-correlated and pool noise instead
        # of silently switching to the old two-source clean-audio shortcut.
        if runtime_mode == COURSE_MODE_TEST_TANK:
            env.update(
                {
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_X_MIN_M": "-2.645",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_X_MAX_M": "2.645",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Y_MIN_M": "-1.270",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Y_MAX_M": "1.270",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Z_MIN_M": "-1.220",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Z_MAX_M": "-0.200",
                }
            )
        elif runtime_mode == "research_pool":
            env.update(
                {
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_X_MIN_M": "-12.5",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_X_MAX_M": "12.5",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Y_MIN_M": "-6.25",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Y_MAX_M": "6.25",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Z_MIN_M": "-3.0",
                    "ROS2_UUV_HYDROPHONE_NOISE_POOL_Z_MAX_M": "-0.2",
                }
            )
        if self._gui_external_mavros_controls_enabled():
            env["SITL_MAVROS_OUTPUT_ENABLE"] = "1"
            env.setdefault("SITL_MAVROS_HOST", "127.0.0.1")
            env.setdefault("SITL_MAVROS_PORT", "14551")
            env["SITL_MAV_GCS_SYSID"] = "1"
            env["SITL_SYSID_MYGCS"] = "1"
            env["ROS2_UUV_SITL_ALLOW_DIRECT_CMD"] = "0"
            env["UUV_GUI_ARM_MODE_COMMAND_PATH"] = "service"
        log_file = None
        try:
            log_path, log_file = open_gui_sim_stack_log()
            env.setdefault(
                "UUV_MJ_THRUSTER_DEBUG_CSV",
                str(log_path.with_name(f"{log_path.stem}_thrusters.csv")),
            )
            if start_purpose == PINGER_HOMING_SIM_PURPOSE:
                launch_args = ["--scene", str(scene_path)]
                launch_args.extend(pinger_sim_launch_args())
            else:
                launch_args = build_sim_launch_preset_args(
                    selected_preset,
                    scene_path=scene_path,
                )
            cmd = build_sim_stack_launch_command(
                self,
                start_script=target.start_script,
                backend=backend,
                extra_args=launch_args,
                enable_stereo_camera=(
                    False if start_purpose == PINGER_HOMING_SIM_PURPOSE else None
                ),
            )
            proc = spawn_sim_stack_process(cmd=cmd, env=env, log_file=log_file)
        except Exception as exc:
            self._set_sim_stack_status(f"sim start failed: {exc}")
            return {"status": self._sim_stack_status}
        finally:
            if log_file is not None:
                _close_quietly(log_file)

        with self._lock:
            self._sim_process = proc
            if start_purpose != PINGER_HOMING_SIM_PURPOSE:
                self._sim_launch_preset_id = selected_preset.preset_id
            self._active_sim_launch_preset_id = selected_preset.preset_id
            self._active_sim_scene = str(scene_path)
            self._sim_stack_status = (
                f"sim: starting {runtime_mode} · {selected_preset.label} ({log_path.name})"
            )
        self.node.push_event(
            "sim stack start requested "
            f"({target.backend}, {runtime_mode}, {selected_preset.profile}/"
            f"{selected_preset.fluid_model}, purpose={start_purpose}): {log_path.name}"
        )
        self._start_process_watcher(
            proc=proc,
            log_path=log_path,
            label="sim stack",
            attr_name="_sim_process",
            status_attr="_sim_stack_status",
        )
        self._schedule_external_mavros_after_sim_reset(proc=proc, log_path=log_path)
        return {
            "status": self._sim_stack_status,
            "log": str(log_path),
            "course_mode": runtime_mode,
            "scene": str(scene_path),
            "purpose": start_purpose,
            "simulation_config": self.simulation_config_payload(),
        }

    def simulation_config_payload(self) -> dict[str, Any]:
        """Return the selected atomic plant preset and the active launch."""

        selected = resolve_sim_launch_preset(self._sim_launch_preset_id)
        active_id = self._active_sim_launch_preset_id if _process_running(self._sim_process) else ""
        return {
            "selected_preset_id": selected.preset_id,
            "selected_label": selected.label,
            "selected_profile": selected.profile,
            "selected_fluid_model": selected.fluid_model,
            "active_preset_id": active_id,
            "active_scene": self._active_sim_scene if active_id else "",
            "presets": sim_launch_presets_payload(),
        }

    def stop_sim_stack(self) -> dict[str, Any]:
        self._terminate_named_process("_mission_process", "_mission_status", "mission", push_event=False)
        self._terminate_named_process(
            "_pinger_homing_process", "_pinger_homing_status", "pinger homing", push_event=False
        )
        self._clear_mission_status_file()
        self._terminate_named_process(
            "_ros_pkg_process", "_ros_pkg_status", "real ROS stack", push_event=False
        )
        self._terminate_named_process("_sim_process", "_sim_stack_status", "sim")
        return {"status": self._sim_stack_status}

    def reset_sim_stack(self) -> dict[str, Any]:
        self._terminate_named_process("_mission_process", "_mission_status", "mission", push_event=False)
        self._terminate_named_process(
            "_pinger_homing_process", "_pinger_homing_status", "pinger homing", push_event=False
        )
        self._clear_mission_status_file()
        self._terminate_named_process("_sim_process", "_sim_stack_status", "sim")
        if not RESET_SIM_STACK_SCRIPT.exists():
            self._set_sim_stack_status(f"sim reset missing: {RESET_SIM_STACK_SCRIPT}")
            return {"status": self._sim_stack_status}
        return self._start_plain_process(
            cmd=[str(RESET_SIM_STACK_SCRIPT), "--wipe-eeprom"],
            cwd=SIM_STACK_DIR,
            label="sim reset",
            log_prefix="web_reset_stack",
            attr_name="_sim_process",
            status_attr="_sim_stack_status",
        )

    def restart_sim_stack(self) -> dict[str, Any]:
        self._clear_stereo_camera_frames()
        self._terminate_named_process("_mission_process", "_mission_status", "mission", push_event=False)
        self._terminate_named_process(
            "_pinger_homing_process", "_pinger_homing_status", "pinger homing", push_event=False
        )
        self._clear_mission_status_file()
        proc = self._sim_process
        if _process_running(proc):
            terminate_process_group(proc, 4.0)
            self.node.push_event("sim restart requested")
            deadline = time.monotonic() + 5.0
            while _process_running(proc) and time.monotonic() < deadline:
                time.sleep(0.05)
            if _process_running(proc):
                self._set_sim_stack_status("sim: stopping")
                return {"status": self._sim_stack_status}
            with self._lock:
                if self._sim_process is proc:
                    self._sim_process = None
        return self.start_sim_stack()

    def camera_config_payload(self) -> dict[str, Any]:
        config = normalize_camera_config(self._camera_config)
        return {
            **config,
            "presets": camera_presets_payload(),
        }

    def configure_camera(self, values: dict[str, Any], *, restart: bool = False) -> dict[str, Any]:
        config = normalize_camera_config(values, strict_preset=True)
        self._camera_config = config
        self.node.push_event(f"Camera profile selected: {config['label']}")
        result: dict[str, Any] = {
            "status": f"camera profile: {config['label']}",
            "camera_config": self.camera_config_payload(),
        }
        if restart:
            result["process"] = self.restart_sim_stack()
        return result

    def build_ros_package(self) -> dict[str, Any]:
        if not ROS_PACKAGE_DIR.exists():
            self._set_ros_pkg_status(f"ros2 build: package missing: {ROS_PACKAGE_DIR}")
            return {"status": self._ros_pkg_status}
        if _process_running(self._ros_build_process):
            self._set_ros_pkg_status("ros2 build: already running")
            return {"status": self._ros_pkg_status}
        return self._start_plain_process(
            cmd=ros_bash_command(ros_package_build_command(), cwd=ROS_WORKSPACE_DIR, include_workspace=False),
            cwd=ROS_WORKSPACE_DIR,
            label="ros2 build",
            log_prefix="web_ros2_build",
            attr_name="_ros_build_process",
            status_attr="_ros_pkg_status",
        )

    def toggle_mavros(self, fcu_url: str | None = None) -> dict[str, Any]:
        if _process_running(self._ros_pkg_process):
            self._terminate_named_process("_ros_pkg_process", "_ros_pkg_status", "mavros")
            return {"status": self._ros_pkg_status, "running": False}
        if fcu_url:
            self._ros_pkg_fcu_url = fcu_url.strip() or ROS_PACKAGE_DEFAULT_FCU_URL
        command = mavros_launch_command(self._ros_pkg_fcu_url)
        result = self._start_plain_process(
            cmd=ros_bash_command(command, cwd=ROS_WORKSPACE_DIR, include_workspace=True),
            cwd=ROS_WORKSPACE_DIR,
            label="mavros",
            log_prefix="web_mavros_pkg",
            attr_name="_ros_pkg_process",
            status_attr="_ros_pkg_status",
        )
        result["running"] = _process_running(self._ros_pkg_process)
        return result

    def _ensure_external_mavros_for_sim(self) -> None:
        if not self._env_flag("UUV_GUI_AUTO_START_MAVROS", True):
            return
        if not self._env_flag("UUV_GUI_USE_EXTERNAL_MAVROS", True):
            return
        if _process_running(self._ros_pkg_process):
            return
        command = mavros_launch_command(self._ros_pkg_fcu_url)
        result = self._start_plain_process(
            cmd=ros_bash_command(command, cwd=ROS_WORKSPACE_DIR, include_workspace=True),
            cwd=ROS_WORKSPACE_DIR,
            label="mavros",
            log_prefix="web_mavros_pkg",
            attr_name="_ros_pkg_process",
            status_attr="_ros_pkg_status",
        )
        if bool(result.get("log")):
            self.node.push_event("real ROS stack auto-start requested for sim stack")

    def _schedule_external_mavros_after_sim_reset(self, *, proc: Any, log_path: Path) -> None:
        if not self._env_flag("UUV_GUI_AUTO_START_MAVROS", True):
            return
        if not self._env_flag("UUV_GUI_USE_EXTERNAL_MAVROS", True):
            return
        thread = threading.Thread(
            target=self._start_external_mavros_after_sim_reset,
            kwargs={"proc": proc, "log_path": log_path},
            name="uuv-web-mavros-after-reset",
            daemon=True,
        )
        thread.start()

    def _start_external_mavros_after_sim_reset(self, *, proc: Any, log_path: Path) -> None:
        # The strict-compat launcher owns UDP 14551 briefly to verify a fresh
        # ArduPilot (1.1) heartbeat.  Starting MAVROS at the old reset marker
        # raced that probe, stole its datagrams, and let MAVROS latch its own
        # 1.191 GCS heartbeat as the remote endpoint.  Wait for the transport
        # readiness marker so the probe has released the socket first.
        deadline = time.monotonic() + 120.0
        transport_ready = False
        matcher = IncrementalLogMatcher(
            log_path,
            (
                "strict compatibility transport READY",
                "startup handoff: SITL and MuJoCo processes are running",
            ),
        )
        while time.monotonic() < deadline:
            with self._lock:
                if self._sim_process is not proc:
                    return
            if proc.poll() is not None:
                return
            if matcher.poll():
                transport_ready = True
                break
            time.sleep(0.10)
        if not transport_ready:
            self.node.push_event("MAVROS delayed start: transport readiness timeout; not starting")
            return
        self._ensure_external_mavros_for_sim()

    def toggle_rviz(self) -> dict[str, Any]:
        if _process_running(self._rviz_process):
            self._terminate_named_process("_rviz_process", "_rviz_status", "rviz")
            return {"status": self._rviz_status, "running": False}
        if not ROS_PACKAGE_RVIZ_CONFIG.exists():
            self._set_rviz_status(f"rviz config missing: {ROS_PACKAGE_RVIZ_CONFIG}")
            return {"status": self._rviz_status}
        try:
            rviz_config = prepare_ros2_rviz_config()
        except Exception as exc:
            self._set_rviz_status(f"rviz config prepare failed: {exc}")
            return {"status": self._rviz_status}
        command = " ".join(shlex.quote(part) for part in ("rviz2", "-d", str(rviz_config)))
        result = self._start_plain_process(
            cmd=ros_bash_command(command, cwd=ROS_WORKSPACE_DIR, include_workspace=False),
            cwd=ROS_WORKSPACE_DIR,
            label="rviz",
            log_prefix="web_rviz",
            attr_name="_rviz_process",
            status_attr="_rviz_status",
        )
        result["running"] = _process_running(self._rviz_process)
        return result

    def start_ping360_view(self) -> dict[str, Any]:
        if _process_running(self._ping360_view_process):
            self._set_ping360_view_status("ping360 view: already open")
            return {"status": self._ping360_view_status}
        try:
            rviz_config = prepare_ping360_rviz_config()
        except Exception as exc:
            self._set_ping360_view_status(f"ping360 rviz config failed: {exc}")
            return {"status": self._ping360_view_status}
        return self._start_plain_process(
            cmd=build_ping360_view_ros_command(rviz_config),
            cwd=SIM_STACK_DIR,
            label="ping360 view",
            log_prefix="web_ping360_view",
            attr_name="_ping360_view_process",
            status_attr="_ping360_view_status",
        )

    def stop_ping360_view(self) -> dict[str, Any]:
        self._terminate_named_process("_ping360_view_process", "_ping360_view_status", "ping360 view")
        return {"status": self._ping360_view_status}

    def start_pinger_homing(self, values: dict[str, Any]) -> dict[str, Any]:
        if _process_running(self._pinger_homing_process):
            self._pinger_homing_status = "pinger homing: already running"
            return {"status": self._pinger_homing_status, "running": True}
        self._terminate_named_process("_mission_process", "_mission_status", "mission", push_event=False)
        clock_barrier = SIM_STACK_DIR / "gui" / "wait_for_mujoco_clock.py"
        homing_selection = str(
            values.get("algorithm", "phase") or "phase"
        ).strip().lower()
        if homing_selection != "phase":
            self._pinger_homing_status = (
                "pinger homing: simulator parity mode supports the physical Phase/odometry "
                "contract only"
            )
            return {"status": self._pinger_homing_status, "running": False}
        requested_navigation_mode = "odometry"
        homing_algorithm = "phase"
        requested_transport = str(values.get("transport", "rc_override") or "rc_override").strip().lower()
        if requested_transport not in {"auto", "rc_override"}:
            self._pinger_homing_status = (
                "pinger homing: the standalone C++ package requires MAVROS RC override; "
                "select RC override"
            )
            return {"status": self._pinger_homing_status, "running": False}
        if not clock_barrier.is_file():
            self._pinger_homing_status = f"pinger homing missing clock barrier: {clock_barrier}"
            return {"status": self._pinger_homing_status, "running": False}

        rate_hz = _float_range(values.get("rate_hz", 30.0), lower=1.0, upper=120.0)
        course_config = load_course_layout_config(COURSE_LAYOUT_CONFIG_PATH)
        test_tank_active = course_config.get("active_mode") == COURSE_MODE_TEST_TANK
        far_forward_limit = 0.55 if test_tank_active else 0.78
        forward_default = 0.48
        forward_fast = min(
            _float_range(
                values.get("forward_fast", forward_default), lower=0.0, upper=0.80
            ),
            far_forward_limit,
        )
        yaw_gain = _float_range(
            values.get("yaw_gain", 0.85),
            lower=0.1,
            upper=2.0,
        )
        yaw_command_limit = _float_range(
            values.get("yaw_command_limit", 0.42),
            lower=0.05,
            upper=0.70,
        )
        rc_pwm_span = _float_range_or_default(
            RC_PWM_SPAN,
            default=400.0,
            lower=50.0,
            upper=700.0,
        )
        probe_pwm_delta = _int_range(
            values.get("probe_pwm_delta", 20),
            lower=1,
            upper=250,
        )
        approach_pwm_delta = _int_range(
            values.get("approach_pwm_delta", 25),
            lower=1,
            upper=250,
        )
        tank_depth_default = TEST_TANK_DEPTH_M if test_tank_active else 11.0
        tank_max_depth_m = _float_range(
            values.get("tank_max_depth_m", tank_depth_default),
            lower=0.5,
            upper=30.0,
        )
        # The old panel shipped with the test-tank depth as its HTML default.
        # Correct that untouched value when the active scene is the 11 m
        # competition tank; explicit non-default operator values are preserved.
        if (
            not test_tank_active
            and abs(tank_max_depth_m - TEST_TANK_DEPTH_M) <= 1.0e-6
        ):
            tank_max_depth_m = 11.0
        success_range_m = _float_range(
            values.get("success_range_m", values.get("stop_range_m", 0.0)),
            lower=0.0,
            upper=10.0,
        )
        success_hold_s = _float_range(
            values.get("success_hold_s", 0.5),
            lower=0.1,
            upper=10.0,
        )
        max_runtime_s = _float_range(
            # The Python-equivalent Phase sequence can legitimately spend
            # more than two minutes on its initial/mirrored probe plus one
            # bounded feedback reprobe in the 11 m competition tank.  A GUI
            # click must not kill that valid sequence at the old 120 s
            # deadline; the controller still has its calibrated-range stop.
            values.get("max_runtime_s", 180.0),
            lower=5.0,
            upper=3600.0,
        )
        clock_barrier_command = " ".join(
            [
                shlex.quote(sys.executable),
                shlex.quote(str(clock_barrier)),
                "--topic",
                "/uuv_mujoco/clock",
                "--min-time",
                "0.5",
                "--stable-samples",
                "3",
                "--timeout",
                "45",
            ]
        )
        launch_arguments = [
            "dry_run:=false",
            # Match the physical controller gate. The web layer may request
            # ALT_HOLD/arm through MAVROS, but the controller itself only
            # observes and verifies the resulting /mavros/state.
            "mode:=ALT_HOLD",
            "auto_arm:=false",
            "auto_mode:=false",
            "use_audio_capture:=false",
            "use_hydrophone_estimator:=true",
            "navigation_mode:=odometry",
            # Match the physical receive-time contract.  The upstream
            # estimator timestamps unstamped PCM and odometry with its own
            # clock; using /clock lets the first audio callback race the new
            # node's clock subscription and creates a zero-time segment.
            "use_sim_time:=false",
            "audio_input_latency_s:=0.0",
            "audio_topic:=/audio",
            "odometry_topic:=/odometry/filtered",
            "imu_topic:=/mavros/imu/data",
            "depth_topic:=/depth/pose",
            "state_topic:=/mavros/state",
            "direction_topic:=/homing/direction",
            "status_topic:=/pinger_homing/status",
            "rc_topic:=/mavros/rc/override",
            f"rate_hz:={rate_hz:.6f}",
            f"forward_max:={forward_fast:.6f}",
            f"yaw_gain:={yaw_gain:.6f}",
            f"yaw_command_limit:={yaw_command_limit:.6f}",
            f"rc_pwm_span:={rc_pwm_span:.6f}",
            f"probe_pwm_delta:={probe_pwm_delta}",
            f"approach_pwm_delta:={approach_pwm_delta}",
            f"tank_max_depth_m:={tank_max_depth_m:.6f}",
            f"success_range_m:={success_range_m:.6f}",
            f"success_hold_s:={success_hold_s:.6f}",
            # The uncalibrated Phase source fit is deliberately independent of
            # MuJoCo ground truth and can be optimistic by a few decimetres.
            # Use the same conservative physical default so completion leaves
            # margin inside the 1.5 m acceptance radius.
            "arrival_radius_m:=0.8",
            f"max_runtime_s:={max_runtime_s:.6f}",
            # Match the uncalibrated physical default. Simulator ground-truth
            # amplitude must not become a hidden success oracle.
            "amplitude_range_constant:=0.0",
            # Run the same ten-second FFT path. The GUI child has no stdin, so
            # it selects only the strongest *qualified* candidate; the real
            # terminal default remains manual selection.
            "auto_select_top:=true",
            "scan_monitor_s:=10.0",
            "scan_min_frequency_hz:=19000.0",
            "scan_max_frequency_hz:=22000.0",
        ]
        launch_command = " ".join(
            [
                "ros2",
                "launch",
                "kmu26_pinger_homing",
                "pinger_homing_real_interactive.launch.py",
                *(shlex.quote(argument) for argument in launch_arguments),
            ]
        )
        command = "\n".join([clock_barrier_command, f"exec {launch_command}"])
        mode_label = "PHASE_REAL_PARITY"
        self.node.push_event(
            f"pinger homing mode: {mode_label} "
            f"(estimator={homing_algorithm}, odometry=required)"
        )
        self.node.push_event(
            "pinger homing tuning: "
            "scan=19000-22000Hz/10s, "
            f"probe_pwm_delta={probe_pwm_delta}, "
            f"approach_pwm_delta={approach_pwm_delta}, "
            "mode=ALT_HOLD, rc=/mavros/rc/override"
        )
        result = self._start_plain_process(
            cmd=ros_bash_command(command, cwd=ROS_WORKSPACE_DIR, include_workspace=True),
            cwd=ROS_WORKSPACE_DIR,
            label="pinger homing",
            log_prefix="web_pinger_homing",
            attr_name="_pinger_homing_process",
            status_attr="_pinger_homing_status",
        )
        result["running"] = _process_running(self._pinger_homing_process)
        result["homing_mode"] = mode_label
        result["navigation_mode"] = requested_navigation_mode
        result["tuning"] = {
            "rc_pwm_span": rc_pwm_span,
            "probe_pwm_delta": probe_pwm_delta,
            "approach_pwm_delta": approach_pwm_delta,
            "scan_monitor_s": 10.0,
            "initial_confirmation_probes": 2,
        }
        if result["running"]:
            self._pinger_homing_status = (
                f"pinger homing: running [{mode_label}] "
                f"(odometry required)"
            )
            result["status"] = self._pinger_homing_status
        return result

    def stop_pinger_homing(self) -> dict[str, Any]:
        self._terminate_named_process("_pinger_homing_process", "_pinger_homing_status", "pinger homing")
        return {"status": self._pinger_homing_status, "running": False}

    def start_vision_processing(self) -> dict[str, Any]:
        if _process_running(self._mission_process):
            self._vision_status = "vision: provided by mission"
            return {"status": self._vision_status, "running": True, "owner": "mission"}
        if _process_running(self._vision_process):
            self._vision_status = "vision: already running"
            return {"status": self._vision_status, "running": True, "owner": "preview"}
        model_path = APP_ROOT / "YOLO" / "best.pt"
        command_parts = [
            "env",
            "OMP_NUM_THREADS=1",
            "MKL_NUM_THREADS=1",
            "OPENBLAS_NUM_THREADS=1",
            "NUMEXPR_NUM_THREADS=1",
            "ros2", "launch", "auv_buoy_vision_control", "laptop_yolo_detection.launch.py",
            "image_topic:=/camera/camera/color/image_raw/compressed",
            # Keep this GUI's camera overlay topic while using the current
            # public launch contract of kmu26_auv_buoy_vision_control.
            "annotated_image_topic:=/vision/buoy/image_annotated/compressed",
            "publish_annotated_image:=true",
            "annotated_jpeg_quality:=80",
            "bbox_topic:=/vision/buoy_bbox",
            # This upstream detector infers on each received frame.  Thread
            # caps above plus the GUI's 4 Hz balanced camera preset keep the
            # CPU-only laptop preview bounded without relying on removed
            # private launch parameters.
            "imgsz:=512",
            "cpu_threads:=1",
            "confidence_threshold:=0.25",
            "target_class_id:=0",
            "target_class_name:=buoy",
            "publish_per_class:=false",
            "show_preview:=false",
        ]
        if model_path.is_file():
            command_parts.append(f"model_path:={model_path}")
        command = " ".join(shlex.quote(part) for part in command_parts)
        result = self._start_plain_process(
            cmd=ros_bash_command(command, cwd=APP_ROOT),
            cwd=APP_ROOT,
            label="vision",
            log_prefix="web_buoy_vision",
            attr_name="_vision_process",
            status_attr="_vision_status",
        )
        result["running"] = _process_running(self._vision_process)
        result["owner"] = "preview"
        return result

    def stop_vision_processing(self) -> dict[str, Any]:
        self._terminate_named_process("_vision_process", "_vision_status", "vision")
        if _process_running(self._mission_process):
            self._vision_status = "vision: mission still running"
            return {"status": self._vision_status, "running": True, "owner": "mission"}
        return {"status": self._vision_status, "running": False, "owner": "none"}

    def vision_preview_running(self) -> bool:
        """Return whether the optional operator-preview detector is owned here."""

        return _process_running(self._vision_process)

    def start_ground_truth_mission(self, values: dict[str, Any]) -> dict[str, Any]:
        if _process_running(self._mission_process):
            self._mission_status = "mission: already running"
            return {"status": self._mission_status, "running": True}
        self._terminate_named_process(
            "_pinger_homing_process", "_pinger_homing_status", "pinger homing", push_event=False
        )
        self._clear_mission_status_file()
        script = SIM_STACK_DIR / "missionFSM" / "run_ground_truth_buoy_controller.py"
        cpp_runner = (
            ROS_WORKSPACE_DIR
            / "install"
            / ROS_PACKAGE_NAME
            / "lib"
            / ROS_PACKAGE_NAME
            / "ground_truth_buoy_fsm"
        )
        course = str(values.get("course", "a")).strip().lower()
        if course not in {"a", "b", "all"}:
            course = "a"
        own_course = str(values.get("own_course", "a" if course == "all" else course)).strip().lower()
        if own_course not in {"a", "b"}:
            own_course = "a"
        max_targets = _int_range(values.get("max_targets", 0 if course == "all" else 1), lower=0, upper=64)
        if course == "all" and max_targets <= 1:
            max_targets = 0
        rate_hz = _float_range(values.get("rate_hz", 30.0), lower=0.5, upper=60.0)
        pinger_forward_fast = _float_range(
            values.get("pinger_forward_fast", 0.55), lower=0.0, upper=1.0)
        pinger_probe_forward = _float_range(
            values.get("pinger_probe_forward", 0.24), lower=0.0, upper=0.60)
        pinger_probe_yaw = _float_range(
            values.get("pinger_probe_yaw", 0.30), lower=0.0, upper=0.80)
        pinger_homing_sway_amplitude = _float_range(
            values.get("pinger_homing_sway_amplitude", 0.0), lower=0.0, upper=0.30)
        pinger_homing_sway_period_s = _float_range(
            values.get("pinger_homing_sway_period_s", 6.0), lower=0.2, upper=30.0)
        pinger_homing_yaw_dither_amplitude = _float_range(
            values.get("pinger_homing_yaw_dither_amplitude", 0.06), lower=0.0, upper=0.20)
        pinger_homing_yaw_dither_period_s = _float_range(
            values.get("pinger_homing_yaw_dither_period_s", 5.0), lower=1.0, upper=30.0)
        pinger_homing_drive_s = _float_range(
            values.get("pinger_homing_drive_s", 0.0), lower=0.0, upper=30.0)
        pinger_homing_pause_s = _float_range(
            values.get("pinger_homing_pause_s", 0.0), lower=0.0, upper=5.0)
        requested_transport = str(values.get("transport", "auto")).strip().lower()
        if requested_transport == "auto":
            external_mavros_controls = self._gui_external_mavros_controls_enabled()
            mission_transport = "rc_override" if external_mavros_controls else "command_override"
        elif requested_transport in {"rc_override", "command_override"}:
            mission_transport = requested_transport
        else:
            external_mavros_controls = self._gui_external_mavros_controls_enabled()
            mission_transport = "rc_override" if external_mavros_controls else "command_override"
        self.node.push_event(f"mission control transport: {mission_transport}")
        simulation_running = self._simulation_runtime_available()
        mission_pose_topic = "/sim/odom" if simulation_running else "/odometry/filtered"
        mission_args = [
            "--controller",
            "mission",
            "--scene",
            str(COURSE_SCENE_PATH),
            "--course",
            course,
            "--own-course",
            own_course,
            "--course-boundary-x",
            "0.0",
            "--course-boundary-margin",
            "0.80",
            "--course-boundary-standoff",
            "0.70",
            "--rate-hz",
            f"{rate_hz:g}",
            "--transport",
            mission_transport,
            "--wait-armed",
            "--rc-pwm-span",
            f"{RC_PWM_SPAN:g}",
            "--mission-log",
            str(values.get("mission_log", "auto") or "auto"),
            "--status-json",
            str(self._mission_status_path),
            "--state-topic",
            "/mavros/state",
            "--yolo-detection-topic",
            "/uuv_mujoco/yolo_buoy_detections",
            "--require-live-status",
            "--live-buoy-timeout-s",
            "3.0",
            "--live-status-timeout-s",
            "3.0",
            "--surface-collect-yolo",
            "--pinger-hydrophone",
            "--pinger-yolo-final-range-m",
            "0.20",
            "--pinger-yolo-near-range-m",
            "0.35",
            "--no-surface-collect-ground-truth",
            "--surface-collector-x-m",
            "-0.011",
            "--surface-collector-y-m",
            "0.0",
            "--surface-collector-z-m",
            "0.360",
            "--surface-collect-x-window-m",
            "0.34",
            "--surface-collect-y-window-m",
            "0.24",
            "--surface-collect-z-window-m",
            "0.30",
            "--score-buoy-tolerance-m",
            "0.35",
            "--score-dump-forward",
            "0.35",
            "--score-dump-pitch",
            "0.55",
        ]
        if course == "all":
            mission_args.extend(["--mission-time-limit-s", "420"])
        if max_targets > 0:
            mission_args.extend(["--max-targets", str(max_targets)])
        if bool(values.get("no_pinger", False if course == "all" else True)):
            mission_args.append("--no-pinger")
        if bool(values.get("nearest_first", True if course == "all" else False)):
            mission_args.append("--nearest-first")
        if bool(values.get("dry_run", False)):
            mission_args.append("--dry-run")

        observation_package_name = "kmu26_mission_fsm"
        observation_launch = (
            ROS_SOURCE_DIR / observation_package_name / "launch" / "mission_fsm_real.launch.py"
        )
        grouped_observation_launch = (
            ROS_SOURCE_DIR
            / "kmu26_control_packages"
            / "kmu26_vision_mission_fsm"
            / "launch"
            / "mission_fsm_real.launch.py"
        )
        if not observation_launch.is_file() and grouped_observation_launch.is_file():
            observation_package_name = "kmu26_vision_mission_fsm"
            observation_launch = grouped_observation_launch
        if observation_launch.is_file():
            no_pinger = bool(values.get("no_pinger", False if course == "all" else True))
            dry_run = bool(values.get("dry_run", False))
            expected_detach_count = 8 if course == "all" and not no_pinger else 7
            expected_net_count = 13 if course == "all" else max(1, max_targets)
            launch_arguments = [
                "use_mission_fsm:=false",
                "use_observation_mission_fsm:=true",
                "use_hydrophone_estimator:=true",
                f"use_vision_detector:={'false' if _process_running(self._vision_process) else 'true'}",
                "use_mission_rviz_visualizer:=true",
                "use_rviz:=false",
                "use_rc_mux:=true",
                "mission_enabled:=true",
                f"dry_run:={'true' if dry_run else 'false'}",
                "wait_armed:=true",
                f"course:={course}",
                f"own_course:={own_course}",
                f"transport:={mission_transport}",
                f"rate_hz:={rate_hz:g}",
                f"pose_topic:={mission_pose_topic}",
                "pose_type:=odometry",
                "state_topic:=/mavros/state",
                "rc_topic:=/mavros/rc/override",
                "camera_compressed_topic:=/camera/camera/color/image_raw/compressed",
                "vision_target_class:=buoy",
                f"observation_use_pinger_first:={'false' if no_pinger else 'true'}",
                "observation_start_surface:=false",
                f"expected_detach_count:={expected_detach_count}",
                f"expected_net_count:={expected_net_count}",
                "vision_expected_target_count:=0",
                "vision_complete_requires_detach_count:=true",
                "score_zone_x:=-6.8",
                "score_zone_y:=0.0",
                "score_zone_radius:=0.8",
                f"pinger_forward_fast:={pinger_forward_fast:g}",
                f"pinger_probe_forward:={pinger_probe_forward:g}",
                f"pinger_probe_yaw:={pinger_probe_yaw:g}",
                f"pinger_homing_sway_amplitude:={pinger_homing_sway_amplitude:g}",
                f"pinger_homing_sway_period_s:={pinger_homing_sway_period_s:g}",
                f"pinger_homing_yaw_dither_amplitude:={pinger_homing_yaw_dither_amplitude:g}",
                f"pinger_homing_yaw_dither_period_s:={pinger_homing_yaw_dither_period_s:g}",
                f"pinger_homing_drive_s:={pinger_homing_drive_s:g}",
                f"pinger_homing_pause_s:={pinger_homing_pause_s:g}",
                f"mission_status_json:={self._mission_status_path}",
            ]
            vision_model_path = APP_ROOT / "YOLO" / "best.pt"
            if vision_model_path.is_file():
                launch_arguments.append(f"vision_model_path:={vision_model_path}")
            if simulation_running:
                # The phase-difference position fit is intentionally disabled:
                # its constant-bearing model goes stale near the source. The
                # simulator's calibrated IQ amplitude supports a separate,
                # robust absolute-range fit; real launches keep that opt-in.
                launch_arguments.append("use_phase_range_position_fusion:=false")
                launch_arguments.append("use_acoustic_position_fusion:=true")
                launch_arguments.append("prefer_upstream_hydrophone_direction:=true")
                launch_arguments.append("prefer_internal_hydrophone_direction:=true")
                # MuJoCo's acoustic amplitude model intentionally saturates at
                # one metre. Freeze the last observable position before that
                # floor, then use the calibrated camera/rake geometry to finish.
                launch_arguments.append("pinger_acoustic_position_lock_range:=1.40")
                launch_arguments.append("pinger_acoustic_position_min_range:=1.05")
                launch_arguments.append("pinger_acoustic_source_depth_z:=-8.645")
                launch_arguments.append("pinger_depth_z:=-8.58")
                # Keep a small forward component while yawing so the single
                # hydrophone receives a useful curved-motion baseline.
                launch_arguments.append("pinger_forward_turn:=0.16")
                launch_arguments.append("pinger_heading_drive_tolerance:=0.18")
                launch_arguments.append("pinger_near_slow_range:=8.0")
                launch_arguments.append("pinger_near_forward:=0.30")
                launch_arguments.append("pinger_final_slow_range:=2.5")
                launch_arguments.append("pinger_final_forward:=0.18")
                launch_arguments.append("pinger_acoustic_crawl_bearing:=0.18")
                launch_arguments.append("pinger_acoustic_crawl_forward:=0.16")
                launch_arguments.append("pinger_acoustic_vertical_zero_range:=1.10")
                launch_arguments.append("pinger_acoustic_vertical_full_range:=2.50")
                launch_arguments.append("pinger_camera_hfov_rad:=1.788")
                launch_arguments.append("vision_horizontal_fov_deg:=102.45")
                launch_arguments.append("vision_capture_aim_offset_x:=0.28")
                launch_arguments.append("pinger_capture_commit_range:=1.10")
                launch_arguments.append("pinger_rake_lane_blend_start:=1.50")
                launch_arguments.append("pinger_rake_lane_full_range:=1.05")
                launch_arguments.append("pinger_position_fit_bearing_tolerance:=0.45")
                launch_arguments.append("pinger_spin_rehome_yaw_rad:=5.75")
                launch_arguments.append("pinger_spin_rehome_max_translation:=0.75")
                launch_arguments.append("pinger_spin_rehome_stop_s:=1.0")
                launch_arguments.append("rc_mux_stale_timeout:=2.0")
            launch_command = " ".join(
                [
                    "ros2",
                    "launch",
                    observation_package_name,
                    "mission_fsm_real.launch.py",
                    *(shlex.quote(argument) for argument in launch_arguments),
                ]
            )
            command = launch_command
            if simulation_running and not dry_run:
                self.node.set_mode("MANUAL")
                self.node.arm(True)
                self.node.push_event("mission start requested MANUAL mode and arm")
                command = "\n".join(
                    [
                        "timeout 4 ros2 service call /mujoco/release_initial_depth_hold "
                        "std_srvs/srv/SetBool '{data: true}' >/dev/null 2>&1 || true",
                        f"exec {launch_command}",
                    ]
                )
            result = self._start_plain_process(
                cmd=ros_bash_command(command, cwd=APP_ROOT),
                cwd=APP_ROOT,
                label="mission",
                log_prefix="web_observation_mission_cpp",
                attr_name="_mission_process",
                status_attr="_mission_status",
            )
            result["running"] = _process_running(self._mission_process)
            return result

        if cpp_runner.is_file() and os.access(cpp_runner, os.X_OK):
            command = " ".join(
                [
                    shlex.quote(str(cpp_runner)),
                    *(shlex.quote(arg) for arg in mission_args),
                ]
            )
            cmd = ros_bash_command(command, cwd=APP_ROOT)
            cwd = APP_ROOT
            log_prefix = "web_gt_mission_cpp"
        else:
            if not script.exists():
                self._mission_status = f"mission missing: {script}"
                return {"status": self._mission_status, "running": False}
            cmd = [sys.executable, str(script), *mission_args]
            cwd = SIM_STACK_DIR
            log_prefix = "web_gt_mission"
        result = self._start_plain_process(
            cmd=cmd,
            cwd=cwd,
            label="mission",
            log_prefix=log_prefix,
            attr_name="_mission_process",
            status_attr="_mission_status",
        )
        result["running"] = _process_running(self._mission_process)
        return result

    def stop_ground_truth_mission(self) -> dict[str, Any]:
        self._terminate_named_process("_mission_process", "_mission_status", "mission")
        self._clear_mission_status_file()
        return {"status": self._mission_status, "running": False}

    def mission_monitor_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "running": _process_running(self._mission_process),
            "process_status": self._mission_status,
            "status_path": str(self._mission_status_path),
        }
        try:
            stat = self._mission_status_path.stat()
            data = json.loads(self._mission_status_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            payload["available"] = False
            return payload
        except Exception as exc:
            payload["available"] = False
            payload["error"] = str(exc)
            return payload
        if isinstance(data, dict):
            payload.update(data)
        payload["available"] = True
        payload["status_age_s"] = max(0.0, time.time() - float(stat.st_mtime))
        return payload

    def _start_plain_process(
        self,
        *,
        cmd: list[str],
        cwd: Path,
        label: str,
        log_prefix: str,
        attr_name: str,
        status_attr: str,
    ) -> dict[str, Any]:
        with self._lock:
            proc = getattr(self, attr_name)
            if _process_running(proc):
                setattr(self, status_attr, f"{label}: already running")
                return {"status": getattr(self, status_attr)}
        log_dir = SIM_STACK_DIR / "logs"
        log_path = allocate_process_log_path(log_dir, log_prefix)
        log_file = None
        try:
            log_file = log_path.open("w", encoding="utf-8")
            proc = subprocess.Popen(
                cmd,
                cwd=str(cwd),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except Exception as exc:
            setattr(self, status_attr, f"{label}: start failed: {exc}")
            return {"status": getattr(self, status_attr)}
        finally:
            if log_file is not None:
                _close_quietly(log_file)
        with self._lock:
            setattr(self, attr_name, proc)
            setattr(self, status_attr, f"{label}: running ({log_path.name})")
        self.node.push_event(f"{label} start requested: {log_path.name}")
        self._start_process_watcher(
            proc=proc,
            log_path=log_path,
            label=label,
            attr_name=attr_name,
            status_attr=status_attr,
        )
        return {"status": getattr(self, status_attr), "log": str(log_path)}

    def _start_process_watcher(self, *, proc: Any, log_path: Path, label: str, attr_name: str, status_attr: str) -> None:
        thread = threading.Thread(
            target=self._watch_process,
            kwargs={
                "proc": proc,
                "log_path": log_path,
                "label": label,
                "attr_name": attr_name,
                "status_attr": status_attr,
            },
            name=f"uuv-web-watch-{label.replace(' ', '-')}",
            daemon=True,
        )
        thread.start()

    def _watch_process(self, *, proc: Any, log_path: Path, label: str, attr_name: str, status_attr: str) -> None:
        terminal_status = None
        if label == "pinger homing":
            matcher = IncrementalLogMatcher(
                log_path,
                ("-> COMPLETE", "-> FAILED_TIMEOUT", "-> FAILED_ESTIMATE"),
            )
            while proc.poll() is None:
                if matcher.poll():
                    terminal_status = _pinger_homing_terminal_log_status(log_path)
                    if terminal_status:
                        # The C++ control tick has published neutral when the
                        # terminal transition is logged, but /pinger_homing/status
                        # is emitted by a separate 5 Hz timer.  Killing the launch
                        # group immediately used to race that timer, so GUI/oracle
                        # subscribers could see CONTACT as the last state even
                        # though the controller reached COMPLETE.  Preserve the
                        # physical ROS contract by allowing at least two status
                        # periods before closing the remaining estimator group.
                        status_grace_deadline = time.monotonic() + 0.50
                        while (
                            proc.poll() is None
                            and time.monotonic() < status_grace_deadline
                        ):
                            time.sleep(0.05)
                        terminate_process_group(proc, 1.0)
                        break
                time.sleep(0.10)
        return_code = proc.wait()
        if label == "pinger homing":
            status = terminal_status or _pinger_homing_exit_status(log_path, return_code)
        else:
            last_line = _last_nonempty_line(log_path)
            if return_code == 0:
                status = f"{label}: exited"
            elif return_code < 0:
                status = f"{label}: stopped"
            else:
                detail = last_line or f"rc={return_code}"
                status = f"{label} failed: {detail}"
        sim_stack_exited = False
        mavros_process = None
        with self._lock:
            if getattr(self, attr_name) is not proc:
                return
            setattr(self, attr_name, None)
            setattr(self, status_attr, status)
            if label == "sim stack":
                # A closed/crashed MuJoCo viewer stops the JSON sensor clock.
                # ArduSub can remain alive in the launcher's process group but
                # advances so slowly that its 1 Hz HEARTBEAT arrives roughly
                # every two minutes.  Do not leave external MAVROS connected
                # to that stale FCU.  Detach the exact MAVROS generation under
                # the lock so a rapid restart cannot be killed by this watcher.
                sim_stack_exited = True
                mavros_process = self._ros_pkg_process
                self._ros_pkg_process = None
                self._ros_pkg_status = "mavros: stopped (sim exited)"
        if sim_stack_exited:
            terminate_exited_process_group(proc)
            terminate_process_group(mavros_process)
            if mavros_process is not None:
                self.node.push_event(self._ros_pkg_status)
        self.node.push_event(status)

    def _terminate_named_process(self, attr_name: str, status_attr: str, label: str, *, push_event: bool = True) -> None:
        proc = getattr(self, attr_name)
        if not _process_running(proc):
            setattr(self, attr_name, None)
            setattr(self, status_attr, f"{label}: stopped")
            return
        terminate_process_group(proc, 4.0)
        setattr(self, attr_name, None)
        setattr(self, status_attr, f"{label}: stopped")
        if push_event:
            self.node.push_event(f"{label} stop requested")

    def _set_sim_stack_status(self, text: str) -> None:
        self._sim_stack_status = text
        self.node.push_event(text)

    def _live_sim_stack_status(self) -> str:
        status = str(self._sim_stack_status)
        if self._simulation_runtime_available():
            return "sim: running"
        if not status.startswith("sim: starting"):
            return status
        return status

    def _simulation_runtime_available(self) -> bool:
        """Recognize a GUI-owned or already-running MuJoCo/SITL stack."""
        if _process_running(self._sim_process):
            return True
        try:
            snap = self.node.snapshot()
        except Exception:
            return False
        return (
            bool(getattr(snap, "connected", False))
            and bool(getattr(snap, "sitl_mavlink_active", False))
            and _fresh_age(getattr(snap, "imu_age_s", math.inf))
            # A fallback/local IMU can remain fresh after Stop Sim. Require
            # the external MAVROS state and SITL diagnostics to be fresh too;
            # otherwise Pinger Start skips simulator startup and waits for a
            # /clock publisher that no longer exists.
            and _fresh_age(getattr(snap, "state_age_s", math.inf), 3.0)
            and _fresh_age(
                getattr(snap, "sitl_mavlink_status_age_s", math.inf), 3.0
            )
        )

    def _set_ros_pkg_status(self, text: str) -> None:
        self._ros_pkg_status = text
        self.node.push_event(text)

    def _set_rviz_status(self, text: str) -> None:
        self._rviz_status = text
        self.node.push_event(text)

    def _set_ping360_view_status(self, text: str) -> None:
        self._ping360_view_status = text
        self.node.push_event(text)

    @staticmethod
    def _env_flag(name: str, default: bool = False) -> bool:
        return env_flag(name, default)

    @staticmethod
    def _arg_present(args: list[str], option: str) -> bool:
        return arg_present(args, option)

    @staticmethod
    def _sim_stack_backend() -> str:
        return sim_stack_backend()

    def _ros_pkg_running(self) -> bool:
        return _process_running(self._ros_pkg_process)

    def _gui_external_mavros_controls_enabled(self) -> bool:
        return self._env_flag("UUV_GUI_USE_EXTERNAL_MAVROS", True)

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

    def _clear_stereo_camera_frames(self) -> None:
        clear_frames = getattr(self.node, "clear_stereo_camera_frames", None)
        if callable(clear_frames):
            clear_frames()

    def _clear_mission_status_file(self) -> None:
        try:
            self._mission_status_path.unlink(missing_ok=True)
        except Exception:
            pass


def _process_running(proc: Any) -> bool:
    return proc is not None and proc.poll() is None


def _fresh_age(value: Any, threshold_s: float = 1.0) -> bool:
    try:
        age = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(age) and age <= float(threshold_s)


def _float_range(value: Any, *, lower: float, upper: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(lower)
    if not math.isfinite(number):
        return float(lower)
    return max(float(lower), min(float(upper), number))


def _float_range_or_default(
    value: Any,
    *,
    default: float,
    lower: float,
    upper: float,
) -> float:
    """Return a finite bounded float without turning malformed input into a limit."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        number = float(default)
    if not math.isfinite(number):
        number = float(default)
    return max(float(lower), min(float(upper), number))


def _int_range(value: Any, *, lower: int, upper: int) -> int:
    return int(round(_float_range(value, lower=float(lower), upper=float(upper))))


def _close_quietly(handle: Any) -> None:
    try:
        handle.close()
    except Exception:
        pass


def _last_nonempty_line(path: Path) -> str:
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception:
        return ""
    for line in reversed(lines):
        text = line.strip()
        if text:
            return text
    return ""


def _pinger_homing_exit_status(path: Path, return_code: int) -> str:
    terminal_status = _pinger_homing_terminal_log_status(path)
    if terminal_status:
        return terminal_status
    if return_code == 0:
        return "pinger homing: exited"
    if return_code < 0:
        return "pinger homing: stopped"
    detail = _last_nonempty_line(path) or f"rc={return_code}"
    return f"pinger homing failed: {detail}"


def _pinger_homing_terminal_log_status(path: Path) -> str:
    try:
        log_text = path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        log_text = ""
    if "pinger homing range success" in log_text:
        return "pinger homing: complete (acoustic range)"
    if "pinger capture confirmed" in log_text or "-> COMPLETE" in log_text:
        return "pinger homing: complete (capture confirmed)"
    if "-> FAILED_TIMEOUT" in log_text:
        return "pinger homing failed: configured runtime limit reached"
    if "-> FAILED_ESTIMATE" in log_text:
        return "pinger homing failed: estimate unavailable"
    return ""


__all__ = ["WebProcessManager"]
