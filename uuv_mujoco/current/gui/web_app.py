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
        self._rc_publish_period_s = 1.0 / 120.0
        self._rc_publish_queued = False
        self._spin_timeout_s = _env_float(
            "UUV_GUI_WEB_SPIN_TIMEOUT_S",
            0.020,
            minimum=0.001,
            maximum=0.100,
        )
        self._control_owner = ""
        self._control_owner_until_wall = 0.0
        self._control_owner_hold_s = 1.25
        self._rc_watchdog_timeout_s = _env_float(
            "UUV_GUI_RC_WATCHDOG_S",
            0.40,
            minimum=0.15,
            maximum=2.0,
        )
        self._rc_watchdog_armed = False
        self._last_rc_receive_wall = -math.inf
        self._last_rc_receive_client = ""
        self._rc_client_sequences: dict[str, tuple[int, float]] = {}
        self._rc_release_queued = False
        self._pinger_was_running = False
        self._pinger_exit_observed_wall: float | None = None
        self._restore_camera_after_pinger = False
        self._restore_vision_after_pinger = False
        self._pinger_activation_lock = threading.Lock()
        self._pinger_activation_generation = 0
        self._pinger_rc_handoff_lock = threading.RLock()
        self._shutdown_lock = threading.Lock()
        self._shutdown_complete = False
        # Ownership is deliberately narrower than "auto_arm was requested".
        # An already-armed vehicle belongs to the operator (or another node)
        # and must remain armed when standalone pinger homing exits.  We own
        # only an auto-arm session that started from a disarmed snapshot.
        self._pinger_auto_arm_owned = False
        self._pinger_auto_arm_candidate_generation: int | None = None
        self._pinger_auto_arm_request_issued_generation: int | None = None
        self._last_pinger_process_check_wall = 0.0
        self.processes = WebProcessManager(node)
        self.replay = WebRcReplayManager(node, self.release_rc)
        self.tools = WebToolFileManager(node)

    def start(self) -> None:
        self._spin_thread = threading.Thread(target=self._spin_loop, name="uuv-web-rclpy", daemon=True)
        self._spin_thread.start()
        # Keep the operator GUI inexpensive at idle.  The annotated preview is
        # still available through the existing Vision toggle, while mission
        # launch owns its detector explicitly.
        auto_vision = os.environ.get("UUV_GUI_VISION_AUTO_START", "0").strip().lower()
        if auto_vision not in {"0", "false", "no", "off"}:
            self.node.set_stereo_camera_display_mode("vision")
            result = self.processes.start_vision_processing()
            self.node.push_event(str(result.get("status", "vision: starting")))
        else:
            self.node.set_stereo_camera_display_mode("raw")

    def stop(self) -> None:
        """Stop children and leave MAVROS in a safe, ownership-respecting state.

        Pinger homing owns an exclusive RC publisher, so its process must stop
        and the GUI publisher must be restored before the final neutral/release
        frame is sent.  If this GUI also confirmed an arm that it requested,
        the disarm is queued after that release.  The current flight mode is
        intentionally never changed during shutdown.
        """
        with self._shutdown_lock:
            if self._shutdown_complete:
                return
            self._shutdown_complete = True

        self._claim_pending_pinger_auto_arm_if_confirmed()
        self._cancel_pinger_activation()
        pinger_active = bool(
            self._pinger_was_running or self.processes.pinger_homing_running()
        )
        self.replay.stop()
        # Stop the exclusive pinger RC source first, but keep MAVROS and the
        # rclpy spin loop alive until the release/disarm queue is drained.
        # WebProcessManager.stop_all() also stops MAVROS, so calling it here
        # would silently strand the queued disarm.
        if pinger_active:
            self.processes.stop_pinger_homing()
        with self._pinger_rc_handoff_lock:
            self._pinger_was_running = False
            self._pinger_exit_observed_wall = None
            if pinger_active:
                self._restore_pinger_rc_publisher("server shutdown")
            self.release_rc()
            disarm_queued = self._enqueue_pinger_owned_disarm("server shutdown")

        # The release and optional disarm must run before the spin loop exits.
        # A barrier makes the ordering deterministic without a timing sleep.
        drained = threading.Event()
        self.enqueue("shutdown_barrier", drained.set)
        spin_alive = bool(self._spin_thread is not None and self._spin_thread.is_alive())
        if spin_alive and threading.current_thread() is not self._spin_thread:
            drained.wait(timeout=2.0)
        else:
            self._drain_commands()

        if disarm_queued:
            confirmation_deadline = time.monotonic() + 2.0
            while time.monotonic() < confirmation_deadline:
                if not bool(self.node.snapshot().armed):
                    self.node.push_event("web pinger homing shutdown disarm confirmed")
                    break
                time.sleep(0.05)
            else:
                self.node.push_event("web pinger homing shutdown disarm confirmation timeout")

        self.processes.stop_all()
        self._stop.set()
        if self._spin_thread is not None and threading.current_thread() is not self._spin_thread:
            self._spin_thread.join(timeout=2.0)

    def enqueue(self, label: str, fn: Callable[[], Any]) -> None:
        self._commands.put((label, fn))

    def enqueue_arm_command(self, value: bool, *, label: str = "arm") -> None:
        """Run backend discovery and arm/disarm on the rclpy spin thread."""

        def command() -> None:
            self.node.probe_backend()
            self.node.arm(bool(value))

        self.enqueue(label, command)

    def enqueue_mode_command(self, mode: str, *, label: str = "mode") -> None:
        """Run backend discovery and mode selection on the rclpy spin thread."""

        requested_mode = str(mode).strip().upper()

        def command() -> None:
            self.node.probe_backend()
            self.node.set_mode(requested_mode)

        self.enqueue(label, command)

    def set_rc(
        self,
        *,
        enabled: bool,
        axes: dict[str, Any],
        client_id: str = "legacy",
        sequence: int | None = None,
    ) -> bool:
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
        should_release = False
        with self._control_lock:
            active_owner = self._control_owner if now <= self._control_owner_until_wall else ""
            if requested_enabled and not has_input and active_owner and active_owner != owner:
                return False
            if not self._record_rc_sequence_locked(owner, sequence, now):
                return False
            if not requested_enabled:
                self._clear_rc_control_locked()
                should_release = True
            elif has_input:
                self._control_enabled = True
                self._axes = clean_axes
                self._control_owner = owner
                self._control_owner_until_wall = now + self._control_owner_hold_s
                if not self._rc_publish_queued:
                    self._rc_publish_queued = True
                    should_enqueue = True
            else:
                self._control_enabled = True
                self._axes = clean_axes
                if active_owner == owner:
                    self._control_owner_until_wall = now + self._control_owner_hold_s
                if not self._rc_publish_queued:
                    self._rc_publish_queued = True
                    should_enqueue = True
            if requested_enabled:
                self._rc_watchdog_armed = True
                self._last_rc_receive_wall = now
                self._last_rc_receive_client = owner
        if should_release:
            self._enqueue_rc_release_once("rc_release")
        elif should_enqueue:
            self.enqueue("rc", self._publish_current_rc)
        return True

    def release_rc(
        self,
        *,
        client_id: str | None = None,
        sequence: int | None = None,
    ) -> bool:
        now = time.monotonic()
        should_enqueue = True
        with self._control_lock:
            if client_id is not None:
                owner = str(client_id or "legacy").strip()[:96] or "legacy"
                active_owner = self._control_owner if now <= self._control_owner_until_wall else ""
                if active_owner and active_owner != owner:
                    return False
                if not self._record_rc_sequence_locked(owner, sequence, now):
                    return False
                # A delayed pagehide packet still advances the sequence so an
                # older active frame cannot revive control.  If the watchdog
                # already completed the release, do not publish it twice.
                should_enqueue = self._control_enabled or self._rc_watchdog_armed
            self._clear_rc_control_locked()
        if should_enqueue:
            self._enqueue_rc_release_once("rc_release")
        return True

    def _record_rc_sequence_locked(
        self,
        owner: str,
        sequence: int | None,
        now: float,
    ) -> bool:
        if sequence is None:
            return True
        previous = self._rc_client_sequences.get(owner)
        if previous is not None and sequence <= previous[0]:
            return False
        if owner not in self._rc_client_sequences and len(self._rc_client_sequences) >= 128:
            oldest_owner = min(
                self._rc_client_sequences,
                key=lambda key: self._rc_client_sequences[key][1],
            )
            self._rc_client_sequences.pop(oldest_owner, None)
        self._rc_client_sequences[owner] = (sequence, now)
        return True

    def _clear_rc_control_locked(self) -> None:
        self._control_enabled = False
        self._axes = {"forward": 0.0, "lateral": 0.0, "heave": 0.0, "yaw": 0.0}
        self._pilot_release_requested = False
        self._rc_publish_queued = False
        self._control_owner = ""
        self._control_owner_until_wall = 0.0
        self._rc_watchdog_armed = False

    def _check_rc_watchdog(self, now: float | None = None) -> bool:
        check_wall = time.monotonic() if now is None else float(now)
        timed_out = False
        last_client = ""
        with self._control_lock:
            if (
                self._rc_watchdog_armed
                and self._control_enabled
                and check_wall - self._last_rc_receive_wall >= self._rc_watchdog_timeout_s
            ):
                last_client = self._last_rc_receive_client
                self._clear_rc_control_locked()
                timed_out = True
        if not timed_out:
            return False
        self._enqueue_rc_release_once("rc_watchdog_release")
        self.node.push_event(
            f"web RC watchdog released stale input from {last_client or 'unknown client'}"
        )
        return True

    def _enqueue_rc_release_once(self, label: str) -> bool:
        with self._control_lock:
            if self._rc_release_queued:
                return False
            self._rc_release_queued = True
        self.enqueue(label, self._publish_rc_release)
        return True

    def status_payload(self) -> dict[str, Any]:
        self.node.probe_backend()
        snap = self.node.snapshot()
        process_payload = self.processes.status_payload()
        self._handle_pinger_process_state(process_payload)
        with self._control_lock:
            rc_receive_age_s = time.monotonic() - self._last_rc_receive_wall
            control = {
                "enabled": self._control_enabled,
                "axes": dict(self._axes),
                "pilot_control_mode": GUI_PILOT_CONTROL_MODE,
                "owner": self._control_owner if time.monotonic() <= self._control_owner_until_wall else "",
                "watchdog": {
                    "armed": self._rc_watchdog_armed,
                    "timeout_s": self._rc_watchdog_timeout_s,
                    "last_rx_age_s": rc_receive_age_s if math.isfinite(rc_receive_age_s) else None,
                    "last_client": self._last_rc_receive_client,
                },
            }
        texts = self._ui_texts(snap, process_status=process_payload)
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
            "processes": process_payload,
            "mission_monitor": process_payload.get("mission_monitor", {}),
            "camera_config": process_payload.get("camera_config", {}),
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

    def set_vision_processing_enabled(self, enabled: bool) -> dict[str, Any]:
        if enabled and self.processes.pinger_homing_running():
            return {
                "status": "vision: paused while pinger homing owns realtime control",
                "running": False,
                "owner": "pinger_homing",
            }
        mode = "vision" if enabled else "raw"
        self.node.set_stereo_camera_display_mode(mode)
        self.node.push_event(f"camera display mode: {mode}")
        if enabled:
            # A detector launched outside this GUI (for example the current
            # auv_buoy_vision_control package under test) already publishes
            # the standard annotated topic.  Reuse it instead of starting a
            # second CPU/GPU inference process merely to change the display.
            camera_status = self.node.stereo_camera_status()
            sources = camera_status.get("sources", {})
            external_vision = sources.get("vision", {}) if isinstance(sources, dict) else {}
            if isinstance(external_vision, dict) and external_vision.get("available", False):
                return {
                    "status": "vision: using external annotated feed",
                    "running": True,
                    "owner": "external",
                }
            return self.processes.start_vision_processing()
        return self.processes.stop_vision_processing()

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

    def start_pinger_homing(self, values: dict[str, Any]) -> dict[str, Any]:
        with self._pinger_rc_handoff_lock:
            # Treat repeated browser clicks and retried HTTP requests as one
            # session.  In particular, do not overwrite the camera/vision
            # restore snapshots or arm ownership while the first run owns RC.
            if self.processes.pinger_homing_running() or self._pinger_was_running:
                return {
                    "status": "pinger homing: already running",
                    "running": True,
                    "idempotent": True,
                }
            sim_start_result: dict[str, Any] | None = None
            if not self.processes.simulation_runtime_available():
                self.node.push_event("pinger homing requested with sim stopped; starting sim stack")
                sim_start_result = self.processes.start_sim_stack(purpose="pinger_homing")
                if not self.processes.simulation_runtime_available():
                    return {
                        "status": (
                            "pinger homing: simulator failed to start: "
                            f"{sim_start_result.get('status', 'unknown error')}"
                        ),
                        "running": False,
                        "sim": sim_start_result,
                    }
            self.release_rc()
            if not self.node.suspend_rc_override_publisher():
                status = "pinger homing: failed to release the web GUI RC publisher"
                self.node.push_event(status)
                return {"status": status, "running": False}
            self.node.push_event("web pilot RC publisher suspended for pinger homing")
            camera_status = self.node.stereo_camera_status()
            self._restore_camera_after_pinger = bool(camera_status.get("enabled", False))
            self._restore_vision_after_pinger = self.processes.vision_preview_running()
            if self._restore_vision_after_pinger:
                self.processes.stop_vision_processing()
                self.node.push_event(
                    "vision preview paused during pinger homing to preserve realtime simulation"
                )
            self.node.set_stereo_camera_enabled(False)
            self.node.push_event("camera feed disabled during pinger homing to preserve realtime simulation")
            self.node.probe_backend()
            # Exercise the same vehicle-state gate as the physical launch.
            # The GUI may request ALT_HOLD through MAVROS, but the C++ node
            # will not emit RC until /mavros/state confirms it.
            mode = "ALT_HOLD"
            auto_arm = bool(values.get("auto_arm", False))
            initially_armed = bool(self.node.snapshot().armed)
            result = self.processes.start_pinger_homing(values)
            if sim_start_result is not None:
                result["sim"] = sim_start_result
            self._pinger_was_running = bool(result.get("running", False))
            if self._pinger_was_running:
                # Merely asking for auto-arm does not establish ownership.  A
                # generation becomes owned only after its arm request is
                # actually issued while disarmed and a later state confirms
                # armed.  This prevents an external/operator arm from being
                # stolen by the GUI session.
                self._pinger_auto_arm_owned = False
                self._pinger_exit_observed_wall = None
                self._schedule_pinger_activation(
                    mode=mode,
                    auto_arm=bool(auto_arm and not initially_armed),
                    own_arm_if_confirmed=bool(auto_arm and not initially_armed),
                )
            else:
                self._pinger_auto_arm_owned = False
                self._cancel_pinger_activation()
                self._restore_pinger_rc_publisher("start failed")
                self._restore_pinger_camera()
                self._restore_pinger_vision()
            return result

    def stop_pinger_homing(self) -> dict[str, Any]:
        with self._pinger_rc_handoff_lock:
            self._claim_pending_pinger_auto_arm_if_confirmed()
            self._cancel_pinger_activation()
            result = self.processes.stop_pinger_homing()
            self._pinger_was_running = False
            self._pinger_exit_observed_wall = None
            self._restore_pinger_rc_publisher("stopped")
            self._restore_pinger_camera()
            self._restore_pinger_vision()
            self.release_rc()
            self._enqueue_pinger_owned_disarm("stopped")
            # ``_publish_rc_release`` sends a neutral frame followed by the
            # MAVROS release frame.  A later neutral override would reclaim
            # the channels immediately after the pinger mux hands them back.
            self.node.push_event("pinger homing stopped; RC override released")
            return result

    def _cancel_pinger_activation(self) -> None:
        with self._pinger_activation_lock:
            self._pinger_activation_generation += 1
            self._pinger_auto_arm_candidate_generation = None
            self._pinger_auto_arm_request_issued_generation = None

    def _schedule_pinger_activation(
        self,
        *,
        mode: str,
        auto_arm: bool,
        own_arm_if_confirmed: bool = False,
    ) -> None:
        """Begin mode/arm retries only after a fresh MAVROS state arrives."""
        with self._pinger_activation_lock:
            self._pinger_activation_generation += 1
            generation = self._pinger_activation_generation
            self._pinger_auto_arm_candidate_generation = (
                generation if own_arm_if_confirmed else None
            )
            self._pinger_auto_arm_request_issued_generation = None

        def activation_current() -> bool:
            with self._pinger_activation_lock:
                return generation == self._pinger_activation_generation

        def wait_for_vehicle() -> None:
            self.node.push_event("web pinger homing waiting for MAVROS vehicle state")
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline and not self._stop.is_set():
                if not activation_current() or not self.processes.pinger_homing_running():
                    return
                snap = self.node.snapshot()
                if (
                    bool(snap.connected)
                    and math.isfinite(snap.state_age_s)
                    and snap.state_age_s < 3.0
                ):
                    self.enqueue_mode_command(mode, label="pinger_homing_mode")
                    self.node.push_event(f"web pinger homing mode requested: {mode}")
                    if not auto_arm:
                        return
                    # A first MAVROS state can arrive while ArduSub is still
                    # completing its startup checks.  A one-shot arm request
                    # was therefore lost even though the GUI reported that it
                    # had been sent.  Retry at a bounded rate until the state
                    # topic confirms arm or the homing process stops.
                    arm_deadline = time.monotonic() + 20.0
                    attempt = 0
                    while time.monotonic() < arm_deadline and not self._stop.is_set():
                        if not activation_current() or not self.processes.pinger_homing_running():
                            return
                        armed_snapshot = self.node.snapshot()
                        if bool(armed_snapshot.armed):
                            owned = self._claim_pinger_auto_arm_after_confirmation(generation)
                            if owned:
                                self.node.push_event(
                                    "web pinger homing arm confirmed and owned after "
                                    f"{attempt} request(s); exit will disarm after RC release"
                                )
                            else:
                                self._clear_pinger_auto_arm_candidate(generation)
                                self.node.push_event(
                                    "web pinger homing observed an external/pre-existing arm; "
                                    "GUI will not disarm it"
                                )
                            return
                        attempt += 1
                        self._enqueue_pinger_auto_arm_request(generation)
                        self.node.push_event(
                            f"web pinger homing auto arm requested ({attempt})"
                        )
                        time.sleep(1.0)
                    if activation_current() and not self._stop.is_set():
                        self.node.push_event("web pinger homing arm confirmation timeout")
                    return
                time.sleep(0.10)
            if activation_current() and not self._stop.is_set():
                self.node.push_event("web pinger homing vehicle wait timeout")

        threading.Thread(
            target=wait_for_vehicle,
            name="uuv-web-pinger-activation",
            daemon=True,
        ).start()

    def _enqueue_pinger_auto_arm_request(self, generation: int) -> None:
        """Issue one session-scoped arm request without claiming ownership."""

        def command() -> None:
            with self._pinger_activation_lock:
                if generation != self._pinger_activation_generation:
                    return
                candidate = self._pinger_auto_arm_candidate_generation == generation
            self.node.probe_backend()
            # Recheck immediately before issuing the command.  If another
            # actor armed since the activation thread's snapshot, this GUI did
            # not cause that arm and must never claim it.
            if bool(self.node.snapshot().armed):
                self._clear_pinger_auto_arm_candidate(generation)
                return
            self.node.arm(True)
            if candidate:
                with self._pinger_activation_lock:
                    if (
                        generation == self._pinger_activation_generation
                        and self._pinger_auto_arm_candidate_generation == generation
                    ):
                        self._pinger_auto_arm_request_issued_generation = generation

        self.enqueue("pinger_homing_auto_arm", command)

    def _claim_pinger_auto_arm_after_confirmation(self, generation: int) -> bool:
        with self._pinger_activation_lock:
            if (
                generation != self._pinger_activation_generation
                or self._pinger_auto_arm_candidate_generation != generation
                or self._pinger_auto_arm_request_issued_generation != generation
            ):
                return False
            self._pinger_auto_arm_owned = True
            self._pinger_auto_arm_candidate_generation = None
            self._pinger_auto_arm_request_issued_generation = None
            return True

    def _clear_pinger_auto_arm_candidate(self, generation: int) -> None:
        with self._pinger_activation_lock:
            if self._pinger_auto_arm_candidate_generation == generation:
                self._pinger_auto_arm_candidate_generation = None
            if self._pinger_auto_arm_request_issued_generation == generation:
                self._pinger_auto_arm_request_issued_generation = None

    def _claim_pending_pinger_auto_arm_if_confirmed(self) -> bool:
        """Promote an issued request when stop/exit wins the retry race."""
        if not bool(self.node.snapshot().armed):
            return False
        with self._pinger_activation_lock:
            generation = self._pinger_auto_arm_request_issued_generation
        if generation is None:
            return False
        return self._claim_pinger_auto_arm_after_confirmation(generation)

    def _handle_pinger_process_state(self, process_payload: dict[str, Any]) -> None:
        with self._pinger_rc_handoff_lock:
            running = bool(process_payload.get("pinger_homing_running", False))
            if running:
                self._pinger_was_running = True
                self._pinger_exit_observed_wall = None
            elif self._pinger_was_running:
                # The wait-for-clock shell hands off to ros2 launch while the
                # simulator is booting.  A single status poll can briefly see
                # that hand-off as stopped; restoring the GUI publisher then
                # races the exclusive pinger mux.  Require a sustained exit.
                now = time.monotonic()
                if self._pinger_exit_observed_wall is None:
                    self._pinger_exit_observed_wall = now
                    return
                if now - self._pinger_exit_observed_wall < 1.0:
                    return
                self._claim_pending_pinger_auto_arm_if_confirmed()
                self._cancel_pinger_activation()
                self._pinger_was_running = False
                self._pinger_exit_observed_wall = None
                self._restore_pinger_rc_publisher("process exited")
                self.release_rc()
                self._enqueue_pinger_owned_disarm("process exited")
                self._restore_pinger_camera()
                self._restore_pinger_vision()

    def _enqueue_pinger_owned_disarm(self, reason: str) -> bool:
        """Release only the arm state this GUI acquired for pinger homing.

        Callers enqueue the RC release first.  Both actions then run in FIFO
        order on the rclpy spin thread, so disarm cannot precede the final
        neutral/release frames and the HTTP request never waits on MAVROS.
        """
        if not self._pinger_auto_arm_owned:
            return False
        self._pinger_auto_arm_owned = False
        self.enqueue_arm_command(False, label="pinger_homing_auto_disarm")
        self.node.push_event(
            f"web pinger homing auto disarm queued after RC release: {reason}"
        )
        return True

    def _restore_pinger_rc_publisher(self, reason: str) -> None:
        if self.node.restore_rc_override_publisher():
            self.node.push_event(f"web pilot RC publisher restored: {reason}")
        else:
            self.node.push_event(f"web pilot RC publisher restore failed: {reason}")

    def _restore_pinger_camera(self) -> None:
        if not self._restore_camera_after_pinger:
            return
        self._restore_camera_after_pinger = False
        self.node.set_stereo_camera_enabled(True)
        self.node.push_event("camera feed restored after pinger homing")

    def _restore_pinger_vision(self) -> None:
        if not self._restore_vision_after_pinger:
            return
        self._restore_vision_after_pinger = False
        result = self.processes.start_vision_processing()
        self.node.set_stereo_camera_display_mode("vision")
        self.node.push_event(str(result.get("status", "vision preview restored")))

    def start_ground_truth_mission(self, values: dict[str, Any]) -> dict[str, Any]:
        self.release_rc()
        self.node.push_event("web pilot control released for mission")
        self.node.probe_backend()
        self.enqueue("mission_auto_arm", lambda: self.node.arm(True))
        self.node.push_event("web mission auto arm requested")
        return self.processes.start_ground_truth_mission(values)

    def stop_ground_truth_mission(self) -> dict[str, Any]:
        return self.processes.stop_ground_truth_mission()

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

    def load_course_layout(self, mode: str | None = None) -> dict[str, Any]:
        return self.tools.load_course_layout(mode)

    def save_course_layout(
        self,
        positions: dict[str, Any],
        *,
        mode: str | None = None,
        robot_xy: Any = None,
        reset: bool = False,
    ) -> dict[str, Any]:
        result = self.tools.save_course_layout(positions, mode=mode, robot_xy=robot_xy, reset=reset)
        if reset:
            result["process"] = self.processes.restart_sim_stack()
        return result

    def _ui_texts(
        self,
        snap: Any,
        *,
        process_status: dict[str, Any],
    ) -> dict[str, str]:
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
            "pinger_homing_status": str(process_status["pinger_homing_status"]),
            "mission_status": str(process_status["mission_status"]),
        }

    def _spin_loop(self) -> None:
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                self._check_rc_watchdog(now)
                self._drain_commands()
                now = time.monotonic()
                with self._control_lock:
                    should_publish = self._control_enabled and now - self._last_rc_publish_wall >= self._rc_publish_period_s
                if should_publish:
                    self._publish_current_rc()
                if now - self._last_pinger_process_check_wall >= 0.10:
                    self._last_pinger_process_check_wall = now
                    self._handle_pinger_process_state(
                        {"pinger_homing_running": self.processes.pinger_homing_running()}
                    )
                rclpy.spin_once(self.node, timeout_sec=self._spin_timeout_s)
            except Exception as exc:  # pragma: no cover - defensive runtime visibility
                self.node.push_event(f"web spin warning: {exc}")
                time.sleep(0.1)

    def _drain_commands(self) -> None:
        for _ in range(64):
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
        if not enabled or self.processes.mission_running() or self.processes.pinger_homing_running():
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
        try:
            if GUI_PILOT_CONTROL_MODE == PILOT_CONTROL_RC_OVERRIDE:
                self.node.publish_rc_override(yaw=0.0, heave=0.0, forward=0.0, lateral=0.0)
                self.node.publish_rc_release()
            else:
                self.node.publish_manual_control(yaw=0.0, heave=0.0, forward=0.0, lateral=0.0)
                self.node.publish_rc_release()
            self.node.push_event("web pilot control released")
        finally:
            with self._control_lock:
                self._rc_release_queued = False

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
        if parsed.path == "/api/stereo/status":
            self._send_json(self.controller.node.stereo_camera_status())
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
        request_path = urlparse(self.path).path
        if (
            request_path == "/api/status"
            or request_path == "/api/stereo/status"
            or request_path in {"/api/stereo/left.jpg", "/api/stereo/right.jpg"}
            or request_path.startswith("/api/rc")
            or (request_path == "/api/command" and status_code == "200")
        ):
            return
        sys.stderr.write(f"[web-gui] {self.address_string()} {fmt % args}\n")

    def _handle_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        command = str(payload.get("command", "rc" if self.path.startswith("/api/rc") else "")).strip().lower()
        if command == "arm":
            value = bool(payload.get("value", True))
            self.controller.enqueue_arm_command(value)
            return {"command": "arm", "armed": value}
        if command == "mode":
            mode = str(payload.get("mode", "")).strip().upper()
            if not mode:
                raise ValueError("missing mode")
            self.controller.enqueue_mode_command(mode)
            return {"command": "mode", "mode": mode}
        if command == "rc":
            axes = payload.get("axes", payload)
            if not isinstance(axes, dict):
                raise ValueError("axes must be an object")
            client_id = str(payload.get("client_id", "legacy"))
            sequence = _optional_rc_sequence(payload.get("seq"))
            enabled = bool(payload.get("enabled", True))
            if bool(payload.get("release", False)) or not enabled:
                accepted = self.controller.release_rc(
                    client_id=client_id,
                    sequence=sequence,
                )
            else:
                accepted = self.controller.set_rc(
                    enabled=True,
                    axes=axes,
                    client_id=client_id,
                    sequence=sequence,
                )
            return {"command": "rc", "accepted": accepted, "seq": sequence}
        if command == "release":
            client_id = str(payload.get("client_id", "")) if "client_id" in payload else None
            sequence = _optional_rc_sequence(payload.get("seq"))
            accepted = self.controller.release_rc(client_id=client_id, sequence=sequence)
            return {"command": "release", "accepted": accepted, "seq": sequence}
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
        if command == "vision_processing_enabled":
            enabled = bool(payload.get("enabled", True))
            return {
                "command": "vision_processing_enabled",
                "enabled": enabled,
                **self.controller.set_vision_processing_enabled(enabled),
            }
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
        if command == "pinger_homing_start":
            values = payload.get("values", payload)
            if not isinstance(values, dict):
                raise ValueError("pinger homing values must be an object")
            return {"command": "pinger_homing_start", **self.controller.start_pinger_homing(values)}
        if command == "pinger_homing_stop":
            return {"command": "pinger_homing_stop", **self.controller.stop_pinger_homing()}
        if command == "gt_mission_start":
            values = payload.get("values", payload)
            if not isinstance(values, dict):
                raise ValueError("mission values must be an object")
            return {"command": "gt_mission_start", **self.controller.start_ground_truth_mission(values)}
        if command == "gt_mission_stop":
            return {"command": "gt_mission_stop", **self.controller.stop_ground_truth_mission()}
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
            mode = str(payload.get("mode", "")).strip() or None
            return {"command": "course_load", **self.controller.load_course_layout(mode)}
        if command == "course_save":
            positions = payload.get("positions", {})
            if not isinstance(positions, dict):
                raise ValueError("positions must be an object")
            return {
                "command": "course_save",
                **self.controller.save_course_layout(
                    positions,
                    mode=str(payload.get("mode", "")).strip() or None,
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
        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            return

    def _send_error_json(self, status: HTTPStatus, message: str) -> None:
        self._send_json({"ok": False, "error": message}, status=status)


def build_handler(controller: WebGuiController, static_dir: Path) -> type[UuvWebHandler]:
    class BoundUuvWebHandler(UuvWebHandler):
        pass

    BoundUuvWebHandler.controller = controller
    BoundUuvWebHandler.static_dir = static_dir
    return BoundUuvWebHandler


def _env_float(name: str, default: float, *, minimum: float, maximum: float) -> float:
    try:
        value = float(str(os.environ.get(name, default)).strip())
    except (TypeError, ValueError):
        value = float(default)
    return max(float(minimum), min(float(maximum), value))


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
    parser.add_argument("--port", type=int, default=8878, help="HTTP bind port")
    parser.add_argument(
        "--image-topic",
        default=os.environ.get("UUV_GUI_STEREO_LEFT_TOPIC", "/camera/camera/color/image_raw/compressed"),
        help="ROS sensor_msgs/Image or CompressedImage topic shown in the web camera panel",
    )
    parser.add_argument(
        "--right-image-topic",
        default=os.environ.get("UUV_GUI_STEREO_RIGHT_TOPIC", "/stereo/right/image_raw"),
        help="Optional right camera topic kept for stereo compatibility",
    )
    parser.add_argument(
        "--mission-status-json",
        default=os.environ.get("UUV_GUI_MISSION_STATUS_JSON", ""),
        help="Mission FSM status JSON shared with the web GUI and RViz visualizer",
    )
    parser.add_argument(
        "--yolo-detection-topic",
        default=os.environ.get("UUV_GUI_YOLO_DETECTION_TOPIC", "/uuv_mujoco/yolo_buoy_detections"),
        help="std_msgs/String YOLO status topic published by the web camera pipeline",
    )
    parser.add_argument(
        "--node-name",
        default=os.environ.get("UUV_WEB_ROS_NODE_NAME", "uuv_web_control_gui"),
        help="ROS node name for the web GUI",
    )
    parser.add_argument("--open-browser", action="store_true", help="Open the web GUI in the default browser")
    args, ros_args = parser.parse_known_args(argv)
    args.ros_args = ros_args
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    os.environ["UUV_GUI_STEREO_LEFT_TOPIC"] = args.image_topic
    os.environ["UUV_GUI_STEREO_RIGHT_TOPIC"] = args.right_image_topic
    os.environ["UUV_GUI_YOLO_DETECTION_TOPIC"] = args.yolo_detection_topic
    if args.mission_status_json:
        os.environ["UUV_GUI_MISSION_STATUS_JSON"] = args.mission_status_json
    rclpy.init(args=args.ros_args or None)
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


def _optional_rc_sequence(value: Any) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError("RC seq must be a non-negative integer")
    try:
        sequence = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("RC seq must be a non-negative integer") from exc
    if sequence < 0 or sequence > 9_007_199_254_740_991 or str(value).strip() != str(sequence):
        raise ValueError("RC seq must be a non-negative integer")
    return sequence


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
