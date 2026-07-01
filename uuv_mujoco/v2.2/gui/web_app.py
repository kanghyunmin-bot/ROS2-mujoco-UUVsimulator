"""Web control surface for the MuJoCo UUV GUI ROS node."""

from __future__ import annotations

import argparse
import copy
import errno
import json
import math
import mimetypes
import os
import queue
import signal
import sys
import threading
import time
import webbrowser
from collections import deque
from dataclasses import fields, is_dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import unquote, urlparse

from .config import (
    AXIS_DEADBAND,
    BACKEND_AUTO,
    BACKEND_MAVROS,
    BACKEND_NONE,
    BACKEND_SIM_BRIDGE,
    GUI_PILOT_CONTROL_MODE,
    PILOT_CONTROL_RC_OVERRIDE,
    ROS_PACKAGE_DIR,
)
from .control_update_pilot_texts import build_pilot_control_texts
from .control_update_telemetry_texts import build_telemetry_texts
from .gui_axis_normalization import clamp_axis
from .gui_rc_axes import gui_rc_to_override_axes
from .models import ControlCommands
from .node import UuvGuiNode
from .runtime import rclpy
from .web_process_manager import WebProcessManager
from .web_rc_replay import WebRcReplayManager
from .web_tool_files import WebToolFileManager


MODE_BUTTONS = ("MANUAL", "STABILIZE", "ALT_HOLD", "GUIDED", "SURFACE", "POSHOLD")
STATIC_DIR = Path(__file__).resolve().parent / "web_static"


class WebGuiController:
    """Thread-safe command and telemetry bridge between HTTP and rclpy."""

    def __init__(self, node: UuvGuiNode) -> None:
        self.node = node
        self._commands: queue.Queue[tuple[str, Callable[[], Any]]] = queue.Queue()
        self._stop = threading.Event()
        self._spin_thread: threading.Thread | None = None
        self._control_lock = threading.Lock()
        self._control_enabled = False
        self._axes = {"forward": 0.0, "lateral": 0.0, "heave": 0.0, "yaw": 0.0}
        self._pilot_release_requested = False
        self._last_rc_publish_wall = 0.0
        self._rc_publish_period_s = 1.0 / 30.0
        self._rc_publish_queued = False
        self._spin_timeout_s = 0.015
        self._control_owner = ""
        self._control_owner_until_wall = 0.0
        self._control_owner_hold_s = 1.25
        self.processes = WebProcessManager(node)
        self.replay = WebRcReplayManager(node, self.release_rc)
        self.tools = WebToolFileManager(node)

    def start(self) -> None:
        self._spin_thread = threading.Thread(target=self._spin_loop, name="uuv-web-rclpy", daemon=True)
        self._spin_thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.replay.stop()
        self.processes.stop_all()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)

    def enqueue(self, label: str, fn: Callable[[], Any]) -> None:
        self._commands.put((label, fn))

    def set_rc(self, *, enabled: bool, axes: dict[str, Any], client_id: str = "legacy") -> None:
        now = time.monotonic()
        requested_enabled = bool(enabled)
        owner = str(client_id or "legacy").strip()[:96] or "legacy"
        clean_axes = {
            "forward": clamp_axis(_float_value(axes.get("forward", 0.0))),
            "lateral": clamp_axis(_float_value(axes.get("lateral", 0.0))),
            "heave": clamp_axis(_float_value(axes.get("heave", 0.0))),
            "yaw": clamp_axis(_float_value(axes.get("yaw", 0.0))),
        }
        has_input = self._has_pilot_input(
            clean_axes["forward"],
            clean_axes["lateral"],
            clean_axes["heave"],
            clean_axes["yaw"],
        )
        should_enqueue = False
        with self._control_lock:
            active_owner = self._control_owner if now <= self._control_owner_until_wall else ""
            if requested_enabled and not has_input and active_owner and active_owner != owner:
                return
            self._control_enabled = requested_enabled
            self._axes = clean_axes
            if not self._control_enabled:
                self._pilot_release_requested = False
                self._rc_publish_queued = False
                self._control_owner = ""
                self._control_owner_until_wall = 0.0
            elif has_input:
                self._control_owner = owner
                self._control_owner_until_wall = now + self._control_owner_hold_s
                if not self._rc_publish_queued:
                    self._rc_publish_queued = True
                    should_enqueue = True
            elif not self._rc_publish_queued:
                if active_owner == owner:
                    self._control_owner_until_wall = now + self._control_owner_hold_s
                self._rc_publish_queued = True
                should_enqueue = True
        if should_enqueue:
            self.enqueue("rc", self._publish_current_rc)

    def release_rc(self) -> None:
        with self._control_lock:
            self._control_enabled = False
            self._axes = {"forward": 0.0, "lateral": 0.0, "heave": 0.0, "yaw": 0.0}
            self._pilot_release_requested = False
            self._rc_publish_queued = False
            self._control_owner = ""
            self._control_owner_until_wall = 0.0
        self.enqueue("rc_release", self._publish_rc_release)

    def status_payload(self) -> dict[str, Any]:
        self.node.probe_backend()
        snap = self.node.snapshot()
        with self._control_lock:
            control = {
                "enabled": self._control_enabled,
                "axes": dict(self._axes),
                "pilot_control_mode": GUI_PILOT_CONTROL_MODE,
                "owner": self._control_owner if time.monotonic() <= self._control_owner_until_wall else "",
            }
        texts = self._ui_texts(snap)
        tool_payload = {}
        tool_payload.update(self.replay.status_payload())
        tool_payload.update(self.tools.status_payload())
        return {
            "telemetry": _telemetry_payload(snap),
            "backend": {
                "label": self.node.backend_label(),
                "mapping": self.node.rc_mapping_summary(),
                "readiness": self.node.control_readiness(snap),
            },
            "control": control,
            "ui": texts,
            "processes": self.processes.status_payload(),
            "camera_config": self.processes.camera_config_payload(),
            "tools": tool_payload,
            "stereo_camera": self.node.stereo_camera_status(),
            "modes": list(MODE_BUTTONS),
            "server": {
                "frontend": "web",
                "ros_package_dir": str(ROS_PACKAGE_DIR),
                "time_wall": time.time(),
            },
        }

    def start_sim_stack(self) -> dict[str, Any]:
        return self.processes.start_sim_stack()

    def stop_sim_stack(self) -> dict[str, Any]:
        return self.processes.stop_sim_stack()

    def reset_sim_stack(self) -> dict[str, Any]:
        return self.processes.reset_sim_stack()

    def configure_camera(self, values: dict[str, Any], *, restart: bool = False) -> dict[str, Any]:
        return self.processes.configure_camera(values, restart=restart)

    def build_ros_package(self) -> dict[str, Any]:
        return self.processes.build_ros_package()

    def toggle_mavros(self, fcu_url: str | None = None) -> dict[str, Any]:
        return self.processes.toggle_mavros(fcu_url)

    def toggle_rviz(self) -> dict[str, Any]:
        return self.processes.toggle_rviz()

    def start_ping360_view(self) -> dict[str, Any]:
        return self.processes.start_ping360_view()

    def stop_ping360_view(self) -> dict[str, Any]:
        return self.processes.stop_ping360_view()

    def load_rc_replay(self, path: str | None = None, rate: str | None = None) -> dict[str, Any]:
        return self.replay.load(path=path, rate=rate)

    def start_rc_replay(self, path: str | None = None, rate: str | None = None) -> dict[str, Any]:
        return self.replay.start(path=path, rate=rate)

    def toggle_rc_replay_pause(self) -> dict[str, Any]:
        return self.replay.toggle_pause()

    def stop_rc_replay(self) -> dict[str, Any]:
        return self.replay.stop()

    def seek_rc_replay(self, time_s: float) -> dict[str, Any]:
        return self.replay.seek(time_s)

    def open_physics_params(self) -> dict[str, Any]:
        return self.tools.open_physics_params()

    def open_course_layout(self) -> dict[str, Any]:
        return self.tools.open_course_layout()

    def read_tool_file(self, kind: str) -> dict[str, Any]:
        return self.tools.read_tool_file(kind)

    def save_tool_file(self, kind: str, content: str) -> dict[str, Any]:
        return self.tools.save_tool_file(kind, content)

    def load_physics_params(self) -> dict[str, Any]:
        return self.tools.load_physics_params()

    def apply_physics_params(self, values: dict[str, Any], *, restart: bool = False) -> dict[str, Any]:
        result = self.tools.apply_physics_params(values, restart=restart)
        if restart:
            result["process"] = self.processes.reset_sim_stack()
        return result

    def load_course_layout(self) -> dict[str, Any]:
        return self.tools.load_course_layout()

    def save_course_layout(
        self,
        positions: dict[str, Any],
        *,
        robot_xy: Any = None,
        reset: bool = False,
    ) -> dict[str, Any]:
        result = self.tools.save_course_layout(positions, robot_xy=robot_xy, reset=reset)
        if reset:
            result["process"] = self.processes.reset_sim_stack()
        return result

    def _ui_texts(self, snap: Any) -> dict[str, str]:
        mode_display = snap.vehicle_mode or snap.mode
        telemetry_texts = build_telemetry_texts(
            snap=snap,
            backend_label=self.node.backend_label(),
            rc_mapping_summary=self.node.rc_mapping_summary(),
            mode_display=mode_display,
            vehicle_info_supported=self.node.vehicle_info_supported(),
        )
        with self._control_lock:
            axes = dict(self._axes)
            enabled = self._control_enabled
        rc_forward, rc_lateral, rc_heave, rc_yaw = gui_rc_to_override_axes(
            forward=axes["forward"],
            lateral=axes["lateral"],
            heave=axes["heave"],
            yaw=axes["yaw"],
        )
        pilot_texts = build_pilot_control_texts(
            snap=snap,
            commands=ControlCommands(
                velocity_forward=0.0,
                velocity_lateral=0.0,
                velocity_heave=0.0,
                velocity_yaw=0.0,
                rc_forward=rc_forward,
                rc_lateral=rc_lateral,
                rc_heave=rc_heave,
                rc_yaw=rc_yaw,
            ),
            rc_mapping_summary=self.node.rc_mapping_summary(),
            control_mode=GUI_PILOT_CONTROL_MODE,
            control_details_visible=False,
            rc_override_enabled=enabled,
        )
        readiness = self.node.control_readiness(snap)
        process_status = self.processes.status_payload()
        return {
            "status": telemetry_texts.status,
            "mode": telemetry_texts.mode,
            "vehicle_summary": telemetry_texts.vehicle_summary,
            "battery": telemetry_texts.battery,
            "pose": telemetry_texts.pose,
            "velocity": telemetry_texts.velocity,
            "imu": telemetry_texts.imu,
            "motion_summary": telemetry_texts.motion_summary,
            "autopilot": telemetry_texts.autopilot,
            "depth_target": telemetry_texts.depth_target,
            "depth_source": telemetry_texts.depth_source,
            "age": telemetry_texts.age,
            "ping360_summary": telemetry_texts.ping360_summary,
            "control_summary": pilot_texts.control_summary,
            "control": pilot_texts.control,
            "rc_override": pilot_texts.rc_override,
            "command_ready": readiness[0] if readiness else "WAIT: vehicle",
            "sim_stack_status": str(process_status["sim_stack_status"]),
            "ros_pkg_status": str(process_status["ros_pkg_status"]),
            "rviz_status": str(process_status["rviz_status"]),
            "ping360_view_status": str(process_status["ping360_view_status"]),
        }

    def _spin_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._drain_commands()
                now = time.monotonic()
                with self._control_lock:
                    should_publish = self._control_enabled and now - self._last_rc_publish_wall >= self._rc_publish_period_s
                if should_publish:
                    self._publish_current_rc()
                rclpy.spin_once(self.node, timeout_sec=self._spin_timeout_s)
            except Exception as exc:  # pragma: no cover - defensive runtime visibility
                self.node.push_event(f"web spin warning: {exc}")
                time.sleep(0.1)

    def _drain_commands(self) -> None:
        for _ in range(32):
            try:
                _label, fn = self._commands.get_nowait()
            except queue.Empty:
                return
            try:
                fn()
            except Exception as exc:  # pragma: no cover - defensive event log path
                self.node.push_event(f"web command failed: {exc}")

    def _publish_current_rc(self) -> None:
        with self._control_lock:
            enabled = self._control_enabled
            axes = dict(self._axes)
            self._rc_publish_queued = False
        if not enabled:
            return
        rc_forward, rc_lateral, rc_heave, rc_yaw = gui_rc_to_override_axes(
            forward=axes["forward"],
            lateral=axes["lateral"],
            heave=axes["heave"],
            yaw=axes["yaw"],
        )
        if self._has_pilot_input(rc_forward, rc_lateral, rc_heave, rc_yaw) and not self._pilot_release_requested:
            self._pilot_release_requested = True
            self.node.request_initial_depth_release_when_armed("web pilot input")
        if GUI_PILOT_CONTROL_MODE == PILOT_CONTROL_RC_OVERRIDE:
            self.node.publish_rc_override(
                yaw=rc_yaw,
                heave=rc_heave,
                forward=rc_forward,
                lateral=rc_lateral,
            )
        else:
            self.node.publish_manual_control(
                yaw=rc_yaw,
                heave=rc_heave,
                forward=rc_forward,
                lateral=rc_lateral,
            )
        self._last_rc_publish_wall = time.monotonic()

    def _publish_rc_release(self) -> None:
        if GUI_PILOT_CONTROL_MODE == PILOT_CONTROL_RC_OVERRIDE:
            self.node.publish_rc_override(yaw=0.0, heave=0.0, forward=0.0, lateral=0.0)
            self.node.publish_rc_release()
        else:
            self.node.publish_manual_control(yaw=0.0, heave=0.0, forward=0.0, lateral=0.0)
            self.node.publish_rc_release()
        self.node.push_event("web pilot control released")

    @staticmethod
    def _has_pilot_input(forward: float, lateral: float, heave: float, yaw: float) -> bool:
        return any(abs(value) > AXIS_DEADBAND for value in (forward, lateral, heave, yaw))


class UuvWebHandler(BaseHTTPRequestHandler):
    controller: WebGuiController
    static_dir: Path

    server_version = "UuvWebGui/1.0"

    def do_GET(self) -> None:  # noqa: N802 - stdlib handler API
        parsed = urlparse(self.path)
        if parsed.path == "/api/status":
            self._send_json(self.controller.status_payload())
            return
        if parsed.path in {"/api/stereo/left.jpg", "/api/stereo/right.jpg"}:
            side = "left" if parsed.path.endswith("/left.jpg") else "right"
            frame = self.controller.node.stereo_camera_frame(side)
            if frame is None:
                self._send_error_json(HTTPStatus.NOT_FOUND, "stereo camera frame not available")
                return
            self._send_binary(frame.data, frame.content_type)
            return
        self._serve_static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802 - stdlib handler API
        parsed = urlparse(self.path)
        if parsed.path not in {"/api/command", "/api/rc"}:
            self._send_error_json(HTTPStatus.NOT_FOUND, "unknown endpoint")
            return
        try:
            payload = self._read_json()
            result = self._handle_command(payload)
        except ValueError as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
            return
        self._send_json({"ok": True, **result})

    def log_message(self, fmt: str, *args: Any) -> None:
        status_code = str(args[1]) if len(args) > 1 else ""
        if (
            self.path == "/api/status"
            or self.path.startswith("/api/rc")
            or (self.path == "/api/command" and status_code == "200")
        ):
            return
        sys.stderr.write(f"[web-gui] {self.address_string()} {fmt % args}\n")

    def _handle_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        command = str(payload.get("command", "rc" if self.path.startswith("/api/rc") else "")).strip().lower()
        if command == "arm":
            value = bool(payload.get("value", True))
            self.controller.node.probe_backend()
            self.controller.enqueue("arm", lambda: self.controller.node.arm(value))
            return {"command": "arm", "armed": value}
        if command == "mode":
            mode = str(payload.get("mode", "")).strip().upper()
            if not mode:
                raise ValueError("missing mode")
            self.controller.node.probe_backend()
            self.controller.enqueue("mode", lambda: self.controller.node.set_mode(mode))
            return {"command": "mode", "mode": mode}
        if command == "rc":
            axes = payload.get("axes", payload)
            if not isinstance(axes, dict):
                raise ValueError("axes must be an object")
            self.controller.set_rc(
                enabled=bool(payload.get("enabled", True)),
                axes=axes,
                client_id=str(payload.get("client_id", "legacy")),
            )
            return {"command": "rc"}
        if command == "release":
            self.controller.release_rc()
            return {"command": "release"}
        if command == "ping360_enabled":
            enabled = bool(payload.get("enabled", True))
            self.controller.enqueue("ping360_enabled", lambda: self.controller.node.publish_ping360_enabled(enabled))
            return {"command": "ping360_enabled", "enabled": enabled}
        if command == "stereo_camera_enabled":
            enabled = bool(payload.get("enabled", True))
            self.controller.enqueue(
                "stereo_camera_enabled",
                lambda: self.controller.node.set_stereo_camera_enabled(enabled),
            )
            return {"command": "stereo_camera_enabled", "enabled": enabled}
        if command == "camera_config":
            values = payload.get("values", payload)
            if not isinstance(values, dict):
                raise ValueError("camera config values must be an object")
            return {
                "command": "camera_config",
                **self.controller.configure_camera(values, restart=bool(payload.get("restart"))),
            }
        if command == "ping360_config":
            config = _ping360_config(payload)
            self.controller.enqueue("ping360_config", lambda: self.controller.node.publish_ping360_config(**config))
            return {"command": "ping360_config"}
        if command == "stack_start":
            return {"command": "stack_start", **self.controller.start_sim_stack()}
        if command == "stack_stop":
            return {"command": "stack_stop", **self.controller.stop_sim_stack()}
        if command == "stack_reset":
            return {"command": "stack_reset", **self.controller.reset_sim_stack()}
        if command == "ros_build":
            return {"command": "ros_build", **self.controller.build_ros_package()}
        if command == "mavros_toggle":
            fcu_url = str(payload.get("fcu_url", "")).strip() or None
            return {"command": "mavros_toggle", **self.controller.toggle_mavros(fcu_url)}
        if command == "rviz_toggle":
            return {"command": "rviz_toggle", **self.controller.toggle_rviz()}
        if command == "ping360_view_start":
            return {"command": "ping360_view_start", **self.controller.start_ping360_view()}
        if command == "ping360_view_stop":
            return {"command": "ping360_view_stop", **self.controller.stop_ping360_view()}
        if command == "rc_replay_load":
            return {
                "command": "rc_replay_load",
                **self.controller.load_rc_replay(
                    path=str(payload.get("path", "")).strip() or None,
                    rate=str(payload.get("rate", "")).strip() or None,
                ),
            }
        if command == "rc_replay_play":
            return {
                "command": "rc_replay_play",
                **self.controller.start_rc_replay(
                    path=str(payload.get("path", "")).strip() or None,
                    rate=str(payload.get("rate", "")).strip() or None,
                ),
            }
        if command == "rc_replay_pause":
            return {"command": "rc_replay_pause", **self.controller.toggle_rc_replay_pause()}
        if command == "rc_replay_stop":
            return {"command": "rc_replay_stop", **self.controller.stop_rc_replay()}
        if command == "rc_replay_seek":
            return {"command": "rc_replay_seek", **self.controller.seek_rc_replay(_float_value(payload.get("time_s", 0.0)))}
        if command == "physics_params":
            return {"command": "physics_params", **self.controller.open_physics_params()}
        if command == "course_layout":
            return {"command": "course_layout", **self.controller.open_course_layout()}
        if command == "physics_load":
            return {"command": "physics_load", **self.controller.load_physics_params()}
        if command == "physics_apply":
            values = payload.get("values", {})
            if not isinstance(values, dict):
                raise ValueError("values must be an object")
            return {
                "command": "physics_apply",
                **self.controller.apply_physics_params(values, restart=bool(payload.get("restart"))),
            }
        if command == "course_load":
            return {"command": "course_load", **self.controller.load_course_layout()}
        if command == "course_save":
            positions = payload.get("positions", {})
            if not isinstance(positions, dict):
                raise ValueError("positions must be an object")
            return {
                "command": "course_save",
                **self.controller.save_course_layout(
                    positions,
                    robot_xy=payload.get("robot_xy"),
                    reset=bool(payload.get("reset")),
                ),
            }
        if command == "tool_read":
            kind = str(payload.get("kind", "")).strip().lower()
            return {"command": "tool_read", **self.controller.read_tool_file(kind)}
        if command == "tool_save":
            kind = str(payload.get("kind", "")).strip().lower()
            content = payload.get("content", "")
            if not isinstance(content, str):
                raise ValueError("content must be a string")
            return {"command": "tool_save", **self.controller.save_tool_file(kind, content)}
        raise ValueError(f"unsupported command: {command}")

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length <= 0:
            return {}
        raw = self.rfile.read(min(length, 1024 * 1024))
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("JSON payload must be an object")
        return payload

    def _serve_static(self, request_path: str) -> None:
        rel = unquote(request_path.lstrip("/"))
        if not rel:
            rel = "index.html"
        path = (self.static_dir / rel).resolve()
        try:
            path.relative_to(self.static_dir.resolve())
        except ValueError:
            self._send_error_json(HTTPStatus.FORBIDDEN, "forbidden path")
            return
        if not path.is_file():
            self._send_error_json(HTTPStatus.NOT_FOUND, "not found")
            return
        content_type, _encoding = mimetypes.guess_type(str(path))
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False, sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_binary(self, data: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_error_json(self, status: HTTPStatus, message: str) -> None:
        self._send_json({"ok": False, "error": message}, status=status)


def build_handler(controller: WebGuiController, static_dir: Path) -> type[UuvWebHandler]:
    class BoundUuvWebHandler(UuvWebHandler):
        pass

    BoundUuvWebHandler.controller = controller
    BoundUuvWebHandler.static_dir = static_dir
    return BoundUuvWebHandler


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Web GUI for MuJoCo UUV telemetry and control")
    parser.add_argument("--namespace", default="/mavros", help="MAVROS namespace to use")
    parser.add_argument(
        "--backend",
        choices=(BACKEND_AUTO, BACKEND_NONE, BACKEND_MAVROS, BACKEND_SIM_BRIDGE, "sim"),
        default=BACKEND_AUTO,
        help="Control/RC compatibility profile",
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP bind port")
    parser.add_argument(
        "--node-name",
        default=os.environ.get("UUV_WEB_ROS_NODE_NAME", "uuv_web_control_gui"),
        help="ROS node name for the web GUI",
    )
    parser.add_argument("--open-browser", action="store_true", help="Open the web GUI in the default browser")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = UuvGuiNode(namespace=args.namespace, backend=args.backend, node_name=args.node_name)
    controller = WebGuiController(node)
    handler_cls = build_handler(controller, STATIC_DIR)
    url = f"http://{args.host}:{args.port}/"
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), handler_cls)
    except OSError as exc:
        node.destroy_node()
        rclpy.shutdown()
        if exc.errno == errno.EADDRINUSE:
            print(f"web-gui already running. Open {url} in your browser.", flush=True)
            return 0
        raise
    controller.start()

    print(f"[web-gui] serving {url}", flush=True)
    if args.open_browser:
        webbrowser.open(url)

    def stop(_signum: int, _frame: Any) -> None:
        threading.Thread(target=httpd.shutdown, name="uuv-web-http-shutdown", daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        httpd.serve_forever(poll_interval=0.2)
    finally:
        controller.release_rc()
        time.sleep(0.05)
        controller.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _float_value(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(result):
        return 0.0
    return result


def _ping360_config(payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "range_m": _float_value(payload.get("range_m", payload.get("requested_range_m", 30.0))),
        "num_steps": int(_float_value(payload.get("num_steps", 400))),
        "gain": int(_float_value(payload.get("gain", payload.get("gain_setting", 1)))),
        "interface_mode": str(payload.get("interface_mode", "ethernet")),
        "frequency_khz": int(_float_value(payload.get("frequency_khz", payload.get("transmit_frequency_khz", 750)))),
        "start_angle_grad": int(_float_value(payload.get("start_angle_grad", 0))),
        "stop_angle_grad": int(_float_value(payload.get("stop_angle_grad", 399))),
    }


def _json_safe_dataclass(value: Any) -> Any:
    if is_dataclass(value):
        return {field.name: _json_safe_value(getattr(value, field.name)) for field in fields(value)}
    return _json_safe_value(value)


def _telemetry_payload(snap: Any) -> dict[str, Any]:
    payload = _json_safe_dataclass(snap)
    payload["roll"] = payload.get("roll_deg")
    payload["pitch"] = payload.get("pitch_deg")
    payload["yaw"] = payload.get("yaw_deg")
    payload["depth"] = payload.get("depth_m")
    payload["lin_vel_xyz"] = payload.get("velocity_xyz")
    return payload


def _json_safe_value(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, deque):
        return [_json_safe_value(item) for item in list(value)]
    if isinstance(value, (list, tuple, set)):
        return [_json_safe_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_safe_value(item) for key, item in value.items()}
    if is_dataclass(value):
        return _json_safe_dataclass(value)
    try:
        return _json_safe_value(copy.deepcopy(value))
    except Exception:
        return str(value)


if __name__ == "__main__":
    raise SystemExit(main())
