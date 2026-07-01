"""Process controls used by the web GUI runtime."""

from __future__ import annotations

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
    RESET_SIM_STACK_SCRIPT,
    ROS_PACKAGE_DEFAULT_FCU_URL,
    ROS_PACKAGE_DIR,
    ROS_PACKAGE_RVIZ_CONFIG,
    ROS_WORKSPACE_DIR,
    SIM_STACK_DIR,
)
from .ping360_view_process import build_ping360_view_ros_command
from .process_env import arg_present, env_flag, sim_stack_backend
from .process_termination import terminate_process_group
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
        self._sim_stack_status = "sim: stopped"
        self._ros_pkg_status = "mavros: stopped"
        self._rviz_status = "rviz: stopped"
        self._ping360_view_status = "ping360 view: closed"
        self._ros_pkg_fcu_url = ROS_PACKAGE_DEFAULT_FCU_URL
        self._camera_config = camera_config_from_owner(self)

    def status_payload(self) -> dict[str, Any]:
        sim_stack_status = self._live_sim_stack_status()
        return {
            "sim_running": _process_running(self._sim_process),
            "ros_build_running": _process_running(self._ros_build_process),
            "mavros_running": _process_running(self._ros_pkg_process),
            "rviz_running": _process_running(self._rviz_process),
            "ping360_view_running": _process_running(self._ping360_view_process),
            "sim_stack_status": sim_stack_status,
            "ros_pkg_status": self._ros_pkg_status,
            "rviz_status": self._rviz_status,
            "ping360_view_status": self._ping360_view_status,
            "ros_pkg_fcu_url": self._ros_pkg_fcu_url,
            "camera_config": self.camera_config_payload(),
        }

    def stop_all(self) -> None:
        for attr_name, status_attr, label in (
            ("_ping360_view_process", "_ping360_view_status", "ping360 view"),
            ("_rviz_process", "_rviz_status", "rviz"),
            ("_ros_pkg_process", "_ros_pkg_status", "mavros"),
            ("_ros_build_process", "_ros_pkg_status", "ros2 build"),
            ("_sim_process", "_sim_stack_status", "sim"),
        ):
            self._terminate_named_process(attr_name, status_attr, label, push_event=False)

    def start_sim_stack(self) -> dict[str, Any]:
        self._clear_stereo_camera_frames()
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

        env = build_gui_sim_stack_env(os.environ, backend=backend, sim_stack_dir=SIM_STACK_DIR)
        log_file = None
        try:
            log_path, log_file = open_gui_sim_stack_log()
            cmd = build_sim_stack_launch_command(self, start_script=target.start_script, backend=backend, extra_args=None)
            proc = spawn_sim_stack_process(cmd=cmd, env=env, log_file=log_file)
        except Exception as exc:
            self._set_sim_stack_status(f"sim start failed: {exc}")
            return {"status": self._sim_stack_status}
        finally:
            if log_file is not None:
                _close_quietly(log_file)

        with self._lock:
            self._sim_process = proc
            self._sim_stack_status = f"sim: starting ({log_path.name})"
        self.node.push_event(f"sim stack start requested ({target.backend}): {log_path.name}")
        self._start_process_watcher(
            proc=proc,
            log_path=log_path,
            label="sim stack",
            attr_name="_sim_process",
            status_attr="_sim_stack_status",
        )
        return {"status": self._sim_stack_status, "log": str(log_path)}

    def stop_sim_stack(self) -> dict[str, Any]:
        self._terminate_named_process("_sim_process", "_sim_stack_status", "sim")
        return {"status": self._sim_stack_status}

    def reset_sim_stack(self) -> dict[str, Any]:
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
            cmd=ros_bash_command(command, cwd=ROS_WORKSPACE_DIR, include_workspace=False),
            cwd=ROS_WORKSPACE_DIR,
            label="mavros",
            log_prefix="web_mavros_pkg",
            attr_name="_ros_pkg_process",
            status_attr="_ros_pkg_status",
        )
        result["running"] = _process_running(self._ros_pkg_process)
        return result

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
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{log_prefix}_{time.strftime('%Y%m%d_%H%M%S')}.log"
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
            setattr(self, status_attr, f"{label}: starting ({log_path.name})")
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
        return_code = proc.wait()
        last_line = _last_nonempty_line(log_path)
        with self._lock:
            if getattr(self, attr_name) is not proc:
                return
            setattr(self, attr_name, None)
            if return_code == 0:
                status = f"{label}: exited"
            elif return_code < 0:
                status = f"{label}: stopped"
            else:
                detail = last_line or f"rc={return_code}"
                status = f"{label} failed: {detail}"
            setattr(self, status_attr, status)
        self.node.push_event(status)

    def _terminate_named_process(self, attr_name: str, status_attr: str, label: str, *, push_event: bool = True) -> None:
        proc = getattr(self, attr_name)
        if not _process_running(proc):
            setattr(self, attr_name, None)
            setattr(self, status_attr, f"{label}: stopped")
            return
        terminate_process_group(proc, 4.0)
        setattr(self, status_attr, f"{label}: stopping")
        if push_event:
            self.node.push_event(f"{label} stop requested")

    def _set_sim_stack_status(self, text: str) -> None:
        self._sim_stack_status = text
        self.node.push_event(text)

    def _live_sim_stack_status(self) -> str:
        status = str(self._sim_stack_status)
        if not status.startswith("sim: starting"):
            return status
        try:
            snap = self.node.snapshot()
        except Exception:
            return status
        if bool(getattr(snap, "connected", False)) and _fresh_age(getattr(snap, "imu_age_s", math.inf)):
            return "sim: running"
        return status

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
        return self._ros_pkg_running() and self._env_flag("UUV_GUI_USE_EXTERNAL_MAVROS", False)

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


def _process_running(proc: Any) -> bool:
    return proc is not None and proc.poll() is None


def _fresh_age(value: Any, threshold_s: float = 1.0) -> bool:
    try:
        age = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(age) and age <= float(threshold_s)


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


__all__ = ["WebProcessManager"]
