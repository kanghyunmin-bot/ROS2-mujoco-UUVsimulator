#!/usr/bin/env python3
"""Collect and judge a running strict MuJoCo/ArduSub/ROS parity stack.

The default invocation is observation-only. Stack startup and arm exercise are
separate explicit options. If arm exercise is requested, cleanup always sends
RC release and disarm, including exception paths.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import traceback
from typing import Any
from urllib import error as urllib_error
from urllib import request as urllib_request


ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = ROOT.parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

from strict_live_parity_contract import (  # noqa: E402
    PARAMETER_NAMES,
    PUBLISHER_OWNERSHIP_RULES,
    REQUIRED_TOPICS,
    assess_arm_event_sequence,
    assess_clock,
    assess_firmware,
    assess_neutral_heave,
    assess_parameters,
    assess_publisher_ownership,
    assess_topic_counts,
    decode_ros_numeric_parameters,
    parse_parameter_text,
    parse_sitl_log,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Verify strict ArduSub-4.1.2, ROS sensor ownership, and optional "
            "safe arm/neutral behavior."
        )
    )
    parser.add_argument("--api-url", default="http://127.0.0.1:8878")
    parser.add_argument("--http-timeout", type=float, default=2.0)
    parser.add_argument("--sitl-log", type=Path)
    parser.add_argument("--observe-seconds", type=float, default=5.0)
    parser.add_argument("--discovery-timeout", type=float, default=12.0)
    parser.add_argument("--parameter-service-timeout", type=float, default=1.5)
    parser.add_argument("--startup-timeout", type=float, default=120.0)
    parser.add_argument(
        "--start-stack",
        action="store_true",
        help="Explicitly ask the Web GUI to start a stopped stack.",
    )
    parser.add_argument(
        "--sim-preset",
        default="",
        help="Optional Web GUI sim preset used only with --start-stack.",
    )
    parser.add_argument(
        "--exercise-arm",
        action="store_true",
        help=(
            "Explicitly call the Web arm API, hold neutral, then always "
            "release and disarm in cleanup."
        ),
    )
    parser.add_argument("--arm-timeout", type=float, default=25.0)
    parser.add_argument("--neutral-observe-seconds", type=float, default=5.0)
    parser.add_argument("--max-neutral-heave-mps", type=float, default=0.20)
    parser.add_argument("--max-neutral-depth-drift-m", type=float, default=0.20)
    parser.add_argument(
        "--report",
        type=Path,
        help="JSON report path (default: generated/strict_live_parity_<time>.json).",
    )
    return parser


class WebApiClient:
    """Small standard-library client for the existing Web GUI API."""

    def __init__(self, base_url: str, *, timeout_s: float) -> None:
        self.base_url = str(base_url).rstrip("/")
        self.timeout_s = max(0.1, float(timeout_s))

    def get_status(self) -> dict[str, Any]:
        return self._request("/api/status")

    def post_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._request("/api/command", payload)

    def post_rc(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._request("/api/rc", payload)

    def _request(
        self,
        path: str,
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        req = urllib_request.Request(
            f"{self.base_url}{path}",
            data=body,
            headers={"Content-Type": "application/json"},
            method="GET" if payload is None else "POST",
        )
        try:
            with urllib_request.urlopen(req, timeout=self.timeout_s) as response:
                decoded = json.loads(response.read().decode("utf-8"))
        except (OSError, ValueError, urllib_error.URLError) as exc:
            raise RuntimeError(f"Web API {path} failed: {exc}") from exc
        if not isinstance(decoded, dict):
            raise RuntimeError(f"Web API {path} returned non-object JSON")
        return decoded


def _status_processes(status: dict[str, Any]) -> dict[str, Any]:
    processes = status.get("processes", {})
    return processes if isinstance(processes, dict) else {}


def _status_telemetry(status: dict[str, Any]) -> dict[str, Any]:
    telemetry = status.get("telemetry", {})
    return telemetry if isinstance(telemetry, dict) else {}


def _status_sim_running(status: dict[str, Any]) -> bool:
    return bool(_status_processes(status).get("sim_running", False))


def _status_armed(status: dict[str, Any]) -> bool:
    return bool(_status_telemetry(status).get("armed", False))


def _status_events(status: dict[str, Any]) -> list[str]:
    events = _status_telemetry(status).get("events", [])
    if not isinstance(events, list):
        return []
    return [str(event) for event in events]


def _new_events_chronological(
    baseline_newest_first: Sequence[str],
    current_newest_first: Sequence[str],
) -> list[str]:
    """Return new Web GUI events in chronological order."""

    current = [str(event) for event in current_newest_first]
    baseline = [str(event) for event in baseline_newest_first]
    if not baseline:
        return list(reversed(current))
    marker = baseline[0]
    try:
        marker_index = current.index(marker)
    except ValueError:
        # The bounded Web event buffer may have evicted the baseline marker.
        return list(reversed(current))
    return list(reversed(current[:marker_index]))


def _wait_for_api_status(
    api: WebApiClient,
    predicate: Callable[[dict[str, Any]], bool],
    *,
    timeout_s: float,
    ros_probe: "RosLiveProbe | None" = None,
) -> dict[str, Any]:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    last_status: dict[str, Any] = {}
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            last_status = api.get_status()
            if predicate(last_status):
                return last_status
            last_error = None
        except Exception as exc:  # keep retrying through short startup gaps
            last_error = exc
        if ros_probe is not None:
            ros_probe.spin_once(0.05)
        time.sleep(0.15)
    if last_error is not None:
        raise RuntimeError(f"Web API status wait failed: {last_error}")
    raise TimeoutError(f"Web API status predicate timed out after {timeout_s:.1f}s")


def _default_report_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "generated" / f"strict_live_parity_{stamp}.json"


def _latest_sitl_log(
    explicit: Path | None,
    *,
    not_before_wall_s: float | None = None,
) -> Path | None:
    if explicit is not None:
        expanded = explicit.expanduser()
        return expanded.resolve() if expanded.exists() else None
    candidates: list[Path] = []
    for pattern in ("sitl_*.log", "docker_sitl_*.log"):
        candidates.extend((ROOT / "logs").glob(pattern))
    candidates = [path for path in candidates if path.is_file()]
    if not candidates:
        return None
    if not_before_wall_s is not None:
        fresh = [
            path
            for path in candidates
            if path.stat().st_mtime >= float(not_before_wall_s) - 5.0
        ]
        if not fresh:
            return None
        candidates = fresh
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _wait_for_sitl_log(
    explicit: Path | None,
    *,
    not_before_wall_s: float | None,
    timeout_s: float,
) -> Path | None:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while True:
        path = _latest_sitl_log(explicit, not_before_wall_s=not_before_wall_s)
        if path is not None:
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                text = ""
            if parse_sitl_log(text).get("firmware") is not None:
                return path
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.2)


def _log_start_wall_s(log_path: Path) -> float | None:
    match = re.search(r"(\d{8})_(\d{6})", log_path.name)
    if match is None:
        return None
    try:
        parsed = datetime.strptime("_".join(match.groups()), "%Y%m%d_%H%M%S")
    except ValueError:
        return None
    return parsed.timestamp()


def _host_path_from_runtime_path(path_text: str) -> Path:
    runtime_path = Path(path_text)
    if str(runtime_path).startswith("/workspace/"):
        return REPOSITORY_ROOT / runtime_path.relative_to("/workspace")
    return runtime_path


def _parameter_artifact_for_log(
    parsed_log: dict[str, Any],
    *,
    log_path: Path,
    not_before_wall_s: float | None,
) -> tuple[Path | None, bool, dict[str, Any]]:
    boundary_wall_s = (
        float(not_before_wall_s)
        if not_before_wall_s is not None
        else _log_start_wall_s(log_path)
    )
    details: dict[str, Any] = {
        "snapshot_marker_observed": bool(
            parsed_log.get("parameter_snapshot_saved", False)
        ),
        "snapshot_parameter_count": parsed_log.get("parameter_snapshot_count"),
        "freshness_not_before_wall_s": boundary_wall_s,
        "artifact_mtime_wall_s": None,
    }
    if not details["snapshot_marker_observed"]:
        return None, False, details
    candidates: list[Path] = []
    binary_text = parsed_log.get("ardusub_binary")
    if binary_text:
        binary_path = _host_path_from_runtime_path(str(binary_text))
        for parent in binary_path.parents:
            if parent.name in {"ardupilot", "ardupilot_sub_stable"}:
                candidates.append(parent / "mav.parm")
                break
    firmware = parsed_log.get("firmware")
    fallback_tree = (
        REPOSITORY_ROOT / "ardupilot_sub_stable"
        if firmware == "ArduSub V4.1.2"
        else REPOSITORY_ROOT / "ardupilot"
    )
    candidates.append(fallback_tree / "mav.parm")
    artifact = next((path for path in candidates if path.is_file()), None)
    if artifact is None:
        return None, False, details
    try:
        artifact_mtime_wall_s = artifact.stat().st_mtime
    except OSError:
        return None, False, details
    details["artifact_mtime_wall_s"] = artifact_mtime_wall_s
    fresh = bool(
        boundary_wall_s is not None
        and artifact_mtime_wall_s >= boundary_wall_s
    )
    return artifact, fresh, details


class RosLiveProbe:
    """Dynamic ROS subscriptions so the tool has no import-time ROS dependency."""

    def __init__(self, *, discovery_timeout_s: float) -> None:
        try:
            import rclpy
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from rosidl_runtime_py.utilities import get_message
        except Exception as exc:
            raise RuntimeError(
                "ROS Python runtime unavailable; source ROS Humble and the "
                "workspace install before running this probe"
            ) from exc

        self._rclpy = rclpy
        self._get_message = get_message
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=[])
        self.node = rclpy.create_node(f"strict_live_parity_probe_{os.getpid()}")
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.subscriptions: list[Any] = []
        self.subscription_errors: dict[str, str] = {}
        self.topic_types: dict[str, list[str]] = {}
        self.counts: dict[str, int] = {topic: 0 for topic in REQUIRED_TOPICS}
        self.clock_nanoseconds: list[int] = []
        self.odom_samples: list[dict[str, float]] = []
        self.state_samples: list[dict[str, Any]] = []
        self.tick_errors: list[str] = []
        self._discover_and_subscribe(float(discovery_timeout_s))

    def _discover_and_subscribe(self, timeout_s: float) -> None:
        deadline = time.monotonic() + max(0.0, timeout_s)
        graph: dict[str, list[str]] = {}
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            graph = {
                name: list(types)
                for name, types in self.node.get_topic_names_and_types()
            }
            if all(topic in graph and graph[topic] for topic in REQUIRED_TOPICS):
                break
        self.topic_types = {
            topic: list(graph.get(topic, [])) for topic in REQUIRED_TOPICS
        }
        for topic in REQUIRED_TOPICS:
            types = self.topic_types.get(topic, [])
            if not types:
                self.subscription_errors[topic] = "topic/type not discovered"
                continue
            try:
                message_type = self._get_message(types[0])
                callback = self._callback_for(topic)
                subscription = self.node.create_subscription(
                    message_type,
                    topic,
                    callback,
                    self.qos,
                )
                self.subscriptions.append(subscription)
            except Exception as exc:
                self.subscription_errors[topic] = str(exc)

    def _callback_for(self, topic: str) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            self.counts[topic] = self.counts.get(topic, 0) + 1
            if topic == "/clock":
                clock = getattr(message, "clock", None)
                if clock is not None:
                    self.clock_nanoseconds.append(
                        int(getattr(clock, "sec", 0)) * 1_000_000_000
                        + int(getattr(clock, "nanosec", 0))
                    )
            elif topic == "/odometry/filtered":
                self._record_odometry(message)
            elif topic == "/mavros/state":
                self.state_samples.append(
                    {
                        "wall_time_s": time.time(),
                        "connected": bool(getattr(message, "connected", False)),
                        "armed": bool(getattr(message, "armed", False)),
                        "mode": str(getattr(message, "mode", "")),
                    }
                )

        return callback

    def _record_odometry(self, message: Any) -> None:
        try:
            position_z = float(message.pose.pose.position.z)
            vertical_speed = float(message.twist.twist.linear.z)
        except (AttributeError, TypeError, ValueError):
            return
        if not math.isfinite(position_z) or not math.isfinite(vertical_speed):
            return
        self.odom_samples.append(
            {
                "wall_time_s": time.time(),
                "position_z_m": position_z,
                "vertical_speed_mps": vertical_speed,
            }
        )

    def reset_samples(self) -> None:
        self.counts = {topic: 0 for topic in REQUIRED_TOPICS}
        self.clock_nanoseconds = []
        self.odom_samples = []
        self.state_samples = []
        self.tick_errors = []

    def spin_once(self, timeout_s: float) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=max(0.0, float(timeout_s)))

    def observe(
        self,
        duration_s: float,
        *,
        tick: Callable[[], None] | None = None,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            self.spin_once(min(0.05, max(0.0, deadline - time.monotonic())))
            if tick is not None:
                try:
                    tick()
                except Exception as exc:
                    self.tick_errors.append(str(exc))
        return self.snapshot()

    def publisher_owners(self) -> dict[str, list[str]]:
        result: dict[str, list[str]] = {}
        for topic in PUBLISHER_OWNERSHIP_RULES:
            owners: set[str] = set()
            try:
                endpoints = self.node.get_publishers_info_by_topic(topic)
            except Exception:
                endpoints = []
            for endpoint in endpoints:
                namespace = str(getattr(endpoint, "node_namespace", "/"))
                name = str(getattr(endpoint, "node_name", ""))
                if not name:
                    continue
                if namespace in {"", "/"}:
                    owners.add(f"/{name}")
                else:
                    owners.add(f"{namespace.rstrip('/')}/{name}")
            result[topic] = sorted(owners)
        return result

    def get_mavros_parameters(
        self,
        names: Sequence[str],
        *,
        service_timeout_s: float,
    ) -> tuple[dict[str, float], dict[str, str]]:
        try:
            from rcl_interfaces.srv import GetParameters
        except Exception as exc:
            return {}, {name: f"ROS parameter API import failed: {exc}" for name in names}

        requested_names = [str(name) for name in names]
        get_client = self.node.create_client(
            GetParameters,
            "/mavros/param/get_parameters",
        )

        def errors_for_all(message: str) -> tuple[dict[str, float], dict[str, str]]:
            return {}, {name: message for name in requested_names}

        def wait_for_future(future: Any, timeout_s: float) -> bool:
            deadline = time.monotonic() + max(0.0, float(timeout_s))
            while not future.done() and time.monotonic() < deadline:
                self.spin_once(min(0.05, max(0.0, deadline - time.monotonic())))
            return bool(future.done())

        try:
            if not get_client.wait_for_service(
                timeout_sec=max(0.0, float(service_timeout_s))
            ):
                return errors_for_all("parameter get service unavailable")

            get_request = GetParameters.Request()
            get_request.names = requested_names
            get_future = get_client.call_async(get_request)
            if not wait_for_future(get_future, service_timeout_s):
                return errors_for_all("live parameter get timed out")
            try:
                get_response = get_future.result()
            except Exception as exc:
                return errors_for_all(f"live parameter get failed: {exc}")
            if get_response is None:
                return errors_for_all("live parameter get returned no response")
            return decode_ros_numeric_parameters(
                requested_names,
                list(get_response.values),
            )
        finally:
            self.node.destroy_client(get_client)

    def snapshot(self) -> dict[str, Any]:
        return {
            "counts": dict(self.counts),
            "clock_nanoseconds": list(self.clock_nanoseconds),
            "odometry_samples": list(self.odom_samples),
            "state_samples": list(self.state_samples),
            "topic_types": dict(self.topic_types),
            "subscription_errors": dict(self.subscription_errors),
            "tick_errors": list(self.tick_errors),
        }

    def close(self) -> None:
        try:
            self.node.destroy_node()
        finally:
            if self._owns_rclpy and self._rclpy.ok():
                self._rclpy.shutdown()


def _collect_parameter_evidence(
    probe: RosLiveProbe,
    *,
    parsed_log: dict[str, Any],
    log_path: Path,
    service_timeout_s: float,
    not_before_wall_s: float | None,
    runtime_active: bool,
) -> dict[str, Any]:
    service_values, service_errors = probe.get_mavros_parameters(
        PARAMETER_NAMES,
        service_timeout_s=service_timeout_s,
    )
    if all(name in service_values for name in PARAMETER_NAMES):
        assessment = assess_parameters(
            service_values,
            source="mavros:/mavros/param/get_parameters",
            source_fresh=True,
        )
        assessment["service_errors"] = service_errors
        assessment["evidence_method"] = "mavros_ros2_get_parameters"
        assessment["evidence_live"] = bool(runtime_active)
        assessment["passed"] = bool(assessment["passed"] and runtime_active)
        return assessment

    log_refresh_error: str | None = None
    try:
        refreshed_log_text = log_path.read_text(encoding="utf-8", errors="replace")
        refreshed_log = parse_sitl_log(refreshed_log_text)
    except OSError as exc:
        refreshed_log = {}
        log_refresh_error = str(exc)

    artifact, artifact_fresh, artifact_details = _parameter_artifact_for_log(
        refreshed_log,
        log_path=log_path,
        not_before_wall_s=not_before_wall_s,
    )
    if artifact is not None:
        values = parse_parameter_text(artifact.read_text(encoding="utf-8", errors="replace"))
        assessment = assess_parameters(
            values,
            source=str(artifact),
            source_fresh=artifact_fresh,
        )
        assessment["service_errors"] = service_errors
        assessment["evidence_method"] = "mavproxy_parameter_snapshot"
        assessment["artifact"] = artifact_details
        assessment["log_refresh_error"] = log_refresh_error
        assessment["evidence_live"] = bool(runtime_active and artifact_fresh is True)
        assessment["passed"] = bool(
            assessment["passed"]
            and runtime_active
            and artifact_fresh is True
        )
        return assessment

    assessment = assess_parameters(
        refreshed_log.get("parameters", parsed_log.get("parameters", {})),
        source=f"{log_path}:configured-startup-values",
        source_fresh=False,
    )
    assessment["service_errors"] = service_errors
    assessment["evidence_method"] = "configured_startup_values_diagnostic_only"
    assessment["artifact"] = artifact_details
    assessment["log_refresh_error"] = log_refresh_error
    assessment["evidence_live"] = False
    # Configured values are useful diagnostics but are not proof of the live FCU state.
    assessment["passed"] = False
    return assessment


def _arm_exercise(
    api: WebApiClient,
    probe: RosLiveProbe,
    *,
    arm_timeout_s: float,
    neutral_observe_s: float,
    max_heave_mps: float,
    max_depth_drift_m: float,
) -> dict[str, Any]:
    before = api.get_status()
    telemetry = _status_telemetry(before)
    if not _status_sim_running(before):
        raise RuntimeError("arm exercise requires a running simulator stack")
    if not bool(telemetry.get("connected", False)):
        raise RuntimeError("arm exercise requires a connected live vehicle state")
    state_age = telemetry.get("state_age_s")
    if state_age is None or float(state_age) > 2.0:
        raise RuntimeError(
            f"arm exercise requires fresh /mavros/state (age={state_age!r}s)"
        )
    if _status_armed(before):
        raise RuntimeError("arm exercise requires an initially disarmed vehicle")
    baseline_events = _status_events(before)
    timeline: list[dict[str, Any]] = []

    timeline.append({"wall_time_s": time.time(), "event": "arm API requested"})
    arm_response = api.post_command({"command": "arm", "value": True})
    armed_status = _wait_for_api_status(
        api,
        _status_armed,
        timeout_s=arm_timeout_s,
        ros_probe=probe,
    )
    timeline.append({"wall_time_s": time.time(), "event": "state armed=True"})
    events_through_armed = _new_events_chronological(
        baseline_events,
        _status_events(armed_status),
    )

    client_id = f"strict-parity-{os.getpid()}"
    sequence = 1
    next_neutral_wall = 0.0

    def publish_neutral() -> None:
        nonlocal sequence, next_neutral_wall
        now = time.monotonic()
        if now < next_neutral_wall:
            return
        api.post_rc(
            {
                "command": "rc",
                "client_id": client_id,
                "seq": sequence,
                "enabled": True,
                "axes": {
                    "forward": 0.0,
                    "lateral": 0.0,
                    "heave": 0.0,
                    "yaw": 0.0,
                },
            }
        )
        sequence += 1
        next_neutral_wall = now + 0.10

    publish_neutral()
    timeline.append(
        {"wall_time_s": time.time(), "event": "neutral RC3=1500 commanded"}
    )
    probe.reset_samples()
    neutral_observation = probe.observe(
        neutral_observe_s,
        tick=publish_neutral,
    )
    final_status = api.get_status()
    events_after_armed = _new_events_chronological(
        _status_events(armed_status),
        _status_events(final_status),
    )
    sequence_events = [
        *events_through_armed,
        "harness state armed=True",
        "harness neutral RC3=1500 commanded",
        *events_after_armed,
    ]
    sequence_assessment = assess_arm_event_sequence(sequence_events)
    neutral_heave = assess_neutral_heave(
        neutral_observation["odometry_samples"],
        max_abs_vertical_speed_mps=max_heave_mps,
        max_vertical_drift_m=max_depth_drift_m,
    )
    remained_armed = _status_armed(final_status)
    ros_armed_observed = any(
        bool(sample.get("armed", False))
        for sample in neutral_observation["state_samples"]
    )
    return {
        "passed": bool(
            sequence_assessment["passed"]
            and neutral_heave["passed"]
            and remained_armed
            and ros_armed_observed
        ),
        "arm_response": arm_response,
        "remained_armed_during_neutral": remained_armed,
        "ros_armed_state_observed": ros_armed_observed,
        "sequence": sequence_assessment,
        "neutral_heave": neutral_heave,
        "neutral_topic_counts": neutral_observation["counts"],
        "neutral_state_samples": neutral_observation["state_samples"],
        "neutral_tick_errors": neutral_observation["tick_errors"],
        "timeline": timeline,
        "cleanup_client_id": client_id,
        "cleanup_sequence": sequence,
    }


def _cleanup_arm(
    api: WebApiClient,
    *,
    arm_report: dict[str, Any] | None,
) -> dict[str, Any]:
    cleanup: dict[str, Any] = {"requested": True, "release": None, "disarm": None}
    client_id = (
        str(arm_report.get("cleanup_client_id"))
        if arm_report and arm_report.get("cleanup_client_id")
        else f"strict-parity-{os.getpid()}"
    )
    sequence = int(arm_report.get("cleanup_sequence", 1)) if arm_report else 1
    try:
        cleanup["release"] = api.post_rc(
            {
                "command": "release",
                "client_id": client_id,
                "seq": sequence,
                "release": True,
                "enabled": False,
            }
        )
    except Exception as exc:
        cleanup["release_error"] = str(exc)
    try:
        cleanup["disarm"] = api.post_command({"command": "arm", "value": False})
        final_status = _wait_for_api_status(
            api,
            lambda status: not _status_armed(status),
            timeout_s=10.0,
        )
        cleanup["confirmed_disarmed"] = not _status_armed(final_status)
    except Exception as exc:
        cleanup["disarm_error"] = str(exc)
        cleanup["confirmed_disarmed"] = False
    cleanup["passed"] = bool(cleanup.get("confirmed_disarmed"))
    return cleanup


def _summarize_baseline(snapshot: dict[str, Any]) -> dict[str, Any]:
    return {
        "counts": snapshot["counts"],
        "clock_sample_count": len(snapshot["clock_nanoseconds"]),
        "clock_first_ns": snapshot["clock_nanoseconds"][0]
        if snapshot["clock_nanoseconds"]
        else None,
        "clock_last_ns": snapshot["clock_nanoseconds"][-1]
        if snapshot["clock_nanoseconds"]
        else None,
        "odometry_sample_count": len(snapshot["odometry_samples"]),
        "state_sample_count": len(snapshot["state_samples"]),
        "topic_types": snapshot["topic_types"],
        "subscription_errors": snapshot["subscription_errors"],
        "tick_errors": snapshot["tick_errors"],
    }


def _required_check_results(report: dict[str, Any], *, exercise_arm: bool) -> list[bool]:
    checks = report.get("checks", {})
    names = ("firmware", "parameters", "clock", "topics", "ownership")
    results = [bool(checks.get(name, {}).get("passed", False)) for name in names]
    if exercise_arm:
        results.append(bool(checks.get("arm_exercise", {}).get("passed", False)))
        results.append(bool(report.get("cleanup", {}).get("arm", {}).get("passed", False)))
    return results


def run(args: argparse.Namespace) -> tuple[dict[str, Any], int]:
    report: dict[str, Any] = {
        "schema_version": 1,
        "started_at": datetime.now().astimezone().isoformat(),
        "mode": {
            "observation_only": not args.start_stack and not args.exercise_arm,
            "start_stack_requested": bool(args.start_stack),
            "arm_exercise_requested": bool(args.exercise_arm),
        },
        "configuration": {
            "api_url": args.api_url,
            "sitl_log": None if args.sitl_log is None else str(args.sitl_log),
            "observe_seconds": float(args.observe_seconds),
            "discovery_timeout_seconds": float(args.discovery_timeout),
            "parameter_service_timeout_seconds": float(
                args.parameter_service_timeout
            ),
            "startup_timeout_seconds": float(args.startup_timeout),
            "sim_preset": args.sim_preset or None,
            "arm_timeout_seconds": float(args.arm_timeout),
            "neutral_observe_seconds": float(args.neutral_observe_seconds),
            "max_neutral_heave_mps": float(args.max_neutral_heave_mps),
            "max_neutral_depth_drift_m": float(
                args.max_neutral_depth_drift_m
            ),
        },
        "checks": {},
        "errors": [],
        "cleanup": {},
    }
    api = WebApiClient(args.api_url, timeout_s=args.http_timeout)
    probe: RosLiveProbe | None = None
    started_by_harness = False
    stack_start_wall: float | None = None
    arm_report: dict[str, Any] | None = None

    try:
        try:
            initial_status = api.get_status()
            report["web_api"] = {
                "available": True,
                "url": args.api_url,
                "initial_sim_running": _status_sim_running(initial_status),
                "initial_armed": _status_armed(initial_status),
            }
        except Exception as exc:
            initial_status = {}
            report["web_api"] = {
                "available": False,
                "url": args.api_url,
                "error": str(exc),
            }

        if args.start_stack and not _status_sim_running(initial_status):
            stack_start_wall = time.time()
            payload: dict[str, Any] = {"command": "stack_start"}
            if args.sim_preset:
                payload["sim_preset"] = args.sim_preset
            report["stack_start"] = api.post_command(payload)
            # From this point the harness owns cleanup even if readiness times out.
            started_by_harness = True
            _wait_for_api_status(
                api,
                _status_sim_running,
                timeout_s=args.startup_timeout,
            )
        elif args.start_stack:
            report["stack_start"] = {"skipped": "stack already running"}

        sitl_log_path = _wait_for_sitl_log(
            args.sitl_log,
            not_before_wall_s=stack_start_wall,
            timeout_s=min(10.0, args.startup_timeout),
        )
        if sitl_log_path is None:
            parsed_log: dict[str, Any] = {}
            report["sitl_log"] = {"available": False}
            report["checks"]["firmware"] = assess_firmware(None)
        else:
            log_text = sitl_log_path.read_text(encoding="utf-8", errors="replace")
            parsed_log = parse_sitl_log(log_text)
            report["sitl_log"] = {
                "available": True,
                "path": str(sitl_log_path),
                "mtime": datetime.fromtimestamp(sitl_log_path.stat().st_mtime)
                .astimezone()
                .isoformat(),
                "ardusub_binary": parsed_log.get("ardusub_binary"),
                "parameter_snapshot_saved": parsed_log.get(
                    "parameter_snapshot_saved",
                    False,
                ),
                "firmware_mentions": parsed_log.get("firmware_mentions", []),
                "configured_parameters": parsed_log.get("parameters", {}),
            }
            report["checks"]["firmware"] = assess_firmware(
                parsed_log.get("firmware")
            )

        probe = RosLiveProbe(discovery_timeout_s=args.discovery_timeout)
        probe.reset_samples()
        baseline = probe.observe(args.observe_seconds)
        owners = probe.publisher_owners()
        report["ros_baseline"] = _summarize_baseline(baseline)
        report["publisher_owners"] = owners
        report["checks"]["clock"] = assess_clock(baseline["clock_nanoseconds"])
        report["checks"]["topics"] = assess_topic_counts(baseline["counts"])
        report["checks"]["ownership"] = assess_publisher_ownership(owners)

        if sitl_log_path is None:
            report["checks"]["parameters"] = assess_parameters(
                {}, source="unavailable", source_fresh=False
            )
        else:
            report["checks"]["parameters"] = _collect_parameter_evidence(
                probe,
                parsed_log=parsed_log,
                log_path=sitl_log_path,
                service_timeout_s=args.parameter_service_timeout,
                not_before_wall_s=stack_start_wall,
                runtime_active=bool(
                    baseline["counts"].get("/clock", 0) > 0
                    and baseline["counts"].get("/mavros/state", 0) > 0
                ),
            )

        if args.exercise_arm:
            if not bool(report.get("web_api", {}).get("available", False)):
                raise RuntimeError("--exercise-arm requires the Web GUI API")
            arm_report = _arm_exercise(
                api,
                probe,
                arm_timeout_s=args.arm_timeout,
                neutral_observe_s=args.neutral_observe_seconds,
                max_heave_mps=args.max_neutral_heave_mps,
                max_depth_drift_m=args.max_neutral_depth_drift_m,
            )
            report["checks"]["arm_exercise"] = arm_report
        else:
            report["checks"]["arm_exercise"] = {
                "passed": None,
                "skipped": "use --exercise-arm for active verification",
            }
    except BaseException as exc:
        report["errors"].append(
            {
                "type": type(exc).__name__,
                "message": str(exc),
                "traceback": traceback.format_exc(),
            }
        )
    finally:
        if args.exercise_arm:
            try:
                report["cleanup"]["arm"] = _cleanup_arm(api, arm_report=arm_report)
            except BaseException as exc:
                report["cleanup"]["arm"] = {
                    "passed": False,
                    "error": f"{type(exc).__name__}: {exc}",
                }
        if probe is not None:
            try:
                probe.close()
                report["cleanup"]["ros_probe_closed"] = True
            except BaseException as exc:
                report["cleanup"]["ros_probe_closed"] = False
                report["cleanup"]["ros_probe_error"] = str(exc)
        if started_by_harness:
            try:
                report["cleanup"]["stack_stop"] = api.post_command(
                    {"command": "stack_stop"}
                )
                stopped_status = _wait_for_api_status(
                    api,
                    lambda status: not _status_sim_running(status),
                    timeout_s=15.0,
                )
                report["cleanup"]["stack_stopped"] = not _status_sim_running(
                    stopped_status
                )
            except BaseException as exc:
                report["cleanup"]["stack_stopped"] = False
                report["cleanup"]["stack_stop_error"] = str(exc)

    report["finished_at"] = datetime.now().astimezone().isoformat()
    required_results = _required_check_results(
        report,
        exercise_arm=bool(args.exercise_arm),
    )
    report["overall_passed"] = bool(required_results and all(required_results))
    report["exit_code"] = 0 if report["overall_passed"] else 1
    return report, int(report["exit_code"])


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    report_path = (args.report or _default_report_path()).expanduser().resolve()
    report, exit_code = run(args)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"strict_live_parity_report={report_path}")
    print(f"strict_live_parity={'PASS' if exit_code == 0 else 'FAIL'}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
