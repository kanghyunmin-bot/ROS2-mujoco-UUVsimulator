import ast
from collections import deque
from copy import deepcopy
from html.parser import HTMLParser
from pathlib import Path
import signal

import rclpy
import pytest
from fastapi import HTTPException
from std_msgs.msg import Float64
from std_msgs.msg import String

from kmu26_auv_web_gui.ros_interface import LocalizationRosNode
from kmu26_auv_web_gui import process_manager as process_manager_module
from kmu26_auv_web_gui.process_manager import ManagedProcess
from kmu26_auv_web_gui.server import _pinger_live_preflight
from kmu26_auv_web_gui.server import _phase_frequency_confirmation_check
from kmu26_auv_web_gui.server import _bounded_int
from kmu26_auv_web_gui.server import _validated_phase_scan_band


ROOT = Path(__file__).resolve().parents[1]
PINGER_ROOT = ROOT.parent / "kmu26_control_packages" / "kmu26_pinger_homing"
VOID_ELEMENTS = {
    "area",
    "base",
    "br",
    "col",
    "embed",
    "hr",
    "img",
    "input",
    "link",
    "meta",
    "param",
    "source",
    "track",
    "wbr",
}


class _LayoutParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.stack: list[tuple[str, str]] = []
        self.section_parents: dict[str, str] = {}
        self.ids: list[str] = []
        self.app_scripts: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        values = dict(attrs)
        element_id = values.get("id") or ""
        if element_id:
            self.ids.append(element_id)
        if tag == "section" and element_id:
            parent_section = next(
                (item_id for item_tag, item_id in reversed(self.stack) if item_tag == "section"),
                "",
            )
            self.section_parents[element_id] = parent_section
        if tag == "script" and "app.js" in (values.get("src") or ""):
            self.app_scripts.append(values["src"] or "")
        if tag not in VOID_ELEMENTS:
            self.stack.append((tag, element_id))

    def handle_startendtag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        self.handle_starttag(tag, attrs)
        if tag not in VOID_ELEMENTS:
            self.handle_endtag(tag)

    def handle_endtag(self, tag: str) -> None:
        for index in range(len(self.stack) - 1, -1, -1):
            if self.stack[index][0] == tag:
                del self.stack[index:]
                return


def test_vision_and_pinger_tabs_are_siblings_and_script_is_loaded_once() -> None:
    parser = _LayoutParser()
    parser.feed((ROOT / "web" / "index.html").read_text(encoding="utf-8"))

    assert parser.section_parents["vision-tab"] == ""
    assert parser.section_parents["pinger-tab"] == ""
    assert len(parser.app_scripts) == 1
    assert len(parser.ids) == len(set(parser.ids))


def test_pinger_parameter_controls_and_css_contract_are_complete() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
    javascript = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
    server = (ROOT / "kmu26_auv_web_gui" / "server.py").read_text(encoding="utf-8")

    assert html.count("data-pinger-param") == 18
    assert 'id="pinger-parameter-reset"' in html
    assert 'id="pinger-parameter-summary"' in html
    assert css.count("{") == css.count("}")
    assert ".vision-log-output" in css
    assert ".pinger-parameter-groups" in css
    assert ".phase-peak-card" in css
    assert "function validatePingerParameters" in javascript
    assert "bindPingerParameterControls();" in javascript
    assert '<option value="phase">PHASE (odometry)</option>' in html
    assert '<option value="no_odom_phase">NO_ODOM_PHASE</option>' in html
    assert '<option value="snr">SNR gradient 3D</option>' in html
    assert (
        'navigation_mode: homingMode === "no_odom_phase" '
        '? "no_odom_phase" : "odometry"' in javascript
    )
    assert 'mode: "STABILIZE"' in javascript
    assert '"mode": "STABILIZE"' in server
    assert 'estimator_mode: homingMode === "no_odom_phase" ? "phase" : homingMode' in javascript
    assert 'odom ${noOdomPhase ? "BYPASSED"' in javascript
    assert 'navigation_mode not in {"odometry", "no_odom_phase"}' in server
    assert '"navigation_mode": navigation_mode' in server
    assert '"/pinger_homing/disabled/odometry"' in server
    assert 'detail="NO_ODOM_PHASE requires estimator_mode=phase"' in server
    assert 'id="pinger-phase-peak-list"' in html
    assert 'id="pinger-phase-manual-confirm"' in html
    assert 'value="15000"' in html
    assert 'value="25000"' in html
    assert (
        'id="pinger-scan-min-frequency" data-pinger-param type="number" '
        'value="15000" min="15000" max="25000"' in html
    )
    assert (
        'id="pinger-scan-max-frequency" data-pinger-param type="number" '
        'value="25000" min="15000" max="25000"' in html
    )
    assert 'value="21164" min="15000" max="25000"' in html
    assert 'id="pinger-reference-frequency"' in html
    assert 'step="any" required' in html
    assert "function phaseFrequenciesMatch" in javascript
    assert '"frequency_hz", 21164.0, 15000.0, 25000.0' in server
    assert '"reference_frequency_hz", 21164.0, 15000.0, 25000.0' in server
    assert 'value="20" min="15" max="25" step="1"' in html
    assert 'value="120" min="20" max="200"' in html
    assert 'postJson("/api/pinger/phase-peaks/select"' in javascript
    assert 'postJson("/api/pinger/phase-peaks/scan"' in javascript
    assert 'use_stamped_audio: $("pinger-use-stamped-audio").checked' in javascript
    assert 'id="pinger-use-stamped-audio" type="checkbox" checked' in html
    assert '"no_odom_probe_pwm_delta": str(' in server
    assert '"no_odom_approach_pwm_delta": str(' in server
    assert '"no_odom_forward_duration_s": str(' in server
    assert 'probe_pwm_delta = _bounded_int(' in server
    assert 'approach_pwm_delta = _bounded_int(' in server
    assert '"phase_peak_confirmed": (' not in server
    assert '"phase_peak_rank": str(' not in server
    assert '"probe_pwm_delta": str(' not in server
    assert '"approach_pwm_delta": str(' not in server
    assert '"approach_duration_s": str(' not in server


def test_ros_bridge_keeps_pinger_subscription_separate_and_owns_no_final_rc() -> None:
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init(args=[])
    node = LocalizationRosNode()
    try:
        subscription_topics = {item.topic_name for item in node.subscriptions}
        publisher_topics = {item.topic_name for item in node.publishers}
        assert "/mission/rc_command" in subscription_topics
        assert "/pinger_homing/status" in subscription_topics
        assert "/pinger_homing/phase_peak_candidates" in subscription_topics
        assert "/pinger_homing/selected_frequency_hz" in subscription_topics
        assert "/pinger_homing/select_frequency_hz" in publisher_topics
        assert "/mavros/rc/override" not in publisher_topics

        candidates = String()
        candidates.data = (
            '{"state":"READY","ready":true,'
            '"suggested_frequency_hz":21164.0,"selected_frequency_hz":null,'
            '"candidates":[{"rank":1,"frequency_hz":21164.0,'
            '"magnitude":0.42,"snr_db":18.5,"quality":0.8,'
            '"support_frames":5,"support":1.0,"selectable":true}]}'
        )
        node._on_phase_peak_candidates(candidates)
        snapshot = node.snapshot()["phase_peak_selection"]
        assert snapshot["scanner_ready"]
        assert snapshot["suggested_frequency_hz"] == 21164.0
        assert snapshot["candidates"][0]["selectable"]
        assert not snapshot["confirmed"]

        request_sequence = node.request_phase_frequency(21163.7, "candidate", 1)
        pending = node.snapshot()["phase_peak_selection"]
        assert not pending["confirmed"]
        selected_message = Float64()
        selected_message.data = 21164.0
        node._on_selected_frequency(selected_message)
        assert node.phase_frequency_selected(request_sequence)
        selected = node.snapshot()["phase_peak_selection"]
        assert selected["confirmed"]
        assert selected["selected_frequency_hz"] == 21164.0
        assert selected["selected_rank"] == 1
    finally:
        node.destroy_node()
        if owned_context and rclpy.ok():
            rclpy.shutdown()


def test_physical_contract_preflight_accepts_valid_data_and_rejects_positive_depth() -> None:
    status = {
        "topics": {
            "odom": {"alive": True},
            "depth": {"alive": True},
            "mavros_state": {"alive": True},
        },
        "mavros_state": {"connected": True, "armed": True, "mode": "ALT_HOLD"},
        "pose": {"z": -1.2},
        "depth": {"z": -1.1},
        "frames": {
            "odom": {"frame_id": "odom", "child_frame_id": "base_link"},
            "depth": {"frame_id": "odom"},
        },
        "graph": {
            "rc_output_publishers": 0,
            "rc_output_publisher_nodes": [],
            "audio_publishers": 1,
            "topic_types": {
                "odom": ["nav_msgs/msg/Odometry"],
                "depth": ["geometry_msgs/msg/PoseWithCovarianceStamped"],
                "mavros_state": ["mavros_msgs/msg/State"],
                "audio": ["audio_common_msgs/msg/AudioData"],
            },
            "services": {"arming": True, "set_mode": True},
        },
        "phase_peak_selection": {
            "selected_frequency_hz": 21164.0,
            "confirmed": True,
            "confirmation_source": "candidate",
        },
    }
    body = {
        "use_hydrophone_estimator": True,
        "use_audio_capture": False,
        "max_runtime_s": 180,
        "arrival_radius_m": 1.5,
        "reference_frequency_hz": 21164.0,
        "phase_peak_confirmed": True,
    }

    valid = _pinger_live_preflight(status, body)
    assert valid["ok"]
    assert len(valid["checks"]) == 19

    positive_depth = deepcopy(status)
    positive_depth["pose"]["z"] = 1.2
    positive_depth["depth"]["z"] = 1.1
    invalid = _pinger_live_preflight(positive_depth, body)
    depth_check = next(item for item in invalid["checks"] if item["name"] == "depth_sign")
    assert not invalid["ok"]
    assert not depth_check["ok"]


def test_no_odom_phase_preflight_bypasses_odom_but_requires_imu_and_depth() -> None:
    status = {
        "topics": {
            "odom": {"alive": False},
            "imu": {"alive": True},
            "depth": {"alive": True},
            "mavros_state": {"alive": True},
        },
        "mavros_state": {"connected": True, "armed": True, "mode": "ALT_HOLD"},
        "pose": {"z": None},
        "depth": {"z": -1.1},
        "frames": {
            "odom": {"frame_id": "", "child_frame_id": ""},
            "depth": {"frame_id": "odom"},
        },
        "graph": {
            "rc_output_publishers": 0,
            "rc_output_publisher_nodes": [],
            "audio_publishers": 1,
            "topic_types": {
                "odom": [],
                "imu": ["sensor_msgs/msg/Imu"],
                "depth": ["geometry_msgs/msg/PoseWithCovarianceStamped"],
                "mavros_state": ["mavros_msgs/msg/State"],
                "audio": ["audio_common_msgs/msg/AudioData"],
            },
            "services": {"arming": True, "set_mode": True},
        },
        "phase_peak_selection": {
            "selected_frequency_hz": 21164.0,
            "confirmed": True,
            "confirmation_source": "manual",
        },
    }
    body = {
        "navigation_mode": "no_odom_phase",
        "estimator_mode": "phase",
        "use_hydrophone_estimator": True,
        "use_audio_capture": False,
        "max_runtime_s": 180,
        "arrival_radius_m": 1.5,
        "reference_frequency_hz": 21164.0,
        "phase_peak_confirmed": True,
    }

    valid = _pinger_live_preflight(status, body)
    assert valid["ok"]
    assert len(valid["checks"]) == 19
    odometry = next(item for item in valid["checks"] if item["name"] == "odometry")
    assert odometry["ok"]
    assert "bypasses" in odometry["detail"]

    stale_imu = deepcopy(status)
    stale_imu["topics"]["imu"]["alive"] = False
    invalid = _pinger_live_preflight(stale_imu, body)
    imu = next(item for item in invalid["checks"] if item["name"] == "imu")
    assert not invalid["ok"]
    assert not imu["ok"]


def test_phase_frequency_requires_explicit_matching_confirmation() -> None:
    status = {
        "phase_peak_selection": {
            "selected_frequency_hz": 21164.0,
            "confirmed": True,
            "confirmation_source": "candidate",
        }
    }
    valid = _phase_frequency_confirmation_check(
        status,
        {
            "estimator_mode": "phase",
            "reference_frequency_hz": 21164.0,
            "phase_peak_confirmed": True,
        },
    )
    assert valid["ok"]

    unconfirmed = _phase_frequency_confirmation_check(
        status,
        {
            "estimator_mode": "phase",
            "reference_frequency_hz": 21164.0,
            "phase_peak_confirmed": False,
        },
    )
    assert not unconfirmed["ok"]

    mismatch = _phase_frequency_confirmation_check(
        status,
        {
            "estimator_mode": "phase",
            "reference_frequency_hz": 22000.0,
            "phase_peak_confirmed": True,
        },
    )
    assert not mismatch["ok"]

    snr = _phase_frequency_confirmation_check(
        {}, {"estimator_mode": "snr", "phase_peak_confirmed": False}
    )
    assert snr["ok"]

    fractional_status = {
        "phase_peak_selection": {
            "selected_frequency_hz": 21164.015617,
            "confirmed": True,
            "confirmation_source": "candidate",
        }
    }
    fractional = _phase_frequency_confirmation_check(
        fractional_status,
        {
            "estimator_mode": "phase",
            "reference_frequency_hz": 21164.015617,
            "phase_peak_confirmed": True,
        },
    )
    assert fractional["ok"]

    edited_after_ack = _phase_frequency_confirmation_check(
        fractional_status,
        {
            "estimator_mode": "phase",
            "reference_frequency_hz": 21164.115617,
            "phase_peak_confirmed": True,
        },
    )
    assert not edited_after_ack["ok"]


def test_web_api_only_passes_declared_pinger_launch_arguments() -> None:
    server_tree = ast.parse(
        (ROOT / "kmu26_auv_web_gui" / "server.py").read_text(encoding="utf-8")
    )
    launch_tree = ast.parse(
        (PINGER_ROOT / "launch" / "pinger_homing_real.launch.py").read_text(
            encoding="utf-8"
        )
    )
    declared = {
        call.args[0].value
        for call in ast.walk(launch_tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[0].value, str)
    }
    start_function = next(
        node
        for node in ast.walk(server_tree)
        if isinstance(node, ast.AsyncFunctionDef) and node.name == "start_pinger"
    )
    launch_dict = next(
        node.value
        for node in ast.walk(start_function)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "launch_args"
            for target in node.targets
        )
        and isinstance(node.value, ast.Dict)
    )
    passed = {
        key.value
        for key in launch_dict.keys
        if isinstance(key, ast.Constant) and isinstance(key.value, str)
    }
    assert passed <= declared, f"undeclared pinger launch args: {sorted(passed - declared)}"
    assert {"mode", "auto_mode"} <= passed

    selector_tree = ast.parse(
        (PINGER_ROOT / "launch" / "phase_peak_selection.launch.py").read_text(
            encoding="utf-8"
        )
    )
    selector_declared = {
        call.args[0].value
        for call in ast.walk(selector_tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[0].value, str)
    }
    scan_function = next(
        node
        for node in ast.walk(server_tree)
        if isinstance(node, ast.AsyncFunctionDef)
        and node.name == "start_phase_peak_scan"
    )
    scan_launch_dict = next(
        node.value
        for node in ast.walk(scan_function)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "launch_args"
            for target in node.targets
        )
        and isinstance(node.value, ast.Dict)
    )
    scan_passed = {
        key.value
        for key in scan_launch_dict.keys
        if isinstance(key, ast.Constant) and isinstance(key.value, str)
    }
    assert scan_passed <= selector_declared, (
        f"undeclared phase scanner launch args: {sorted(scan_passed - selector_declared)}"
    )


def test_pwm_deltas_are_exact_integer_launch_values_without_silent_clamping() -> None:
    probe = _bounded_int({"probe_pwm_delta": 20}, "probe_pwm_delta", 20, 15, 25)
    approach = _bounded_int(
        {"approach_pwm_delta": 120}, "approach_pwm_delta", 120, 20, 200
    )
    assert str(probe) == "20"
    assert str(approach) == "120"
    for exact_probe in (15, 20, 25):
        assert _bounded_int(
            {"probe_pwm_delta": exact_probe}, "probe_pwm_delta", 20, 15, 25
        ) == exact_probe
    for rejected_probe in (14, 26):
        with pytest.raises(HTTPException):
            _bounded_int(
                {"probe_pwm_delta": rejected_probe},
                "probe_pwm_delta",
                20,
                15,
                25,
            )
    with pytest.raises(HTTPException):
        _bounded_int(
            {"probe_pwm_delta": 20.5}, "probe_pwm_delta", 20, 15, 25
        )


def test_phase_scan_band_is_narrowing_only_inside_15_to_25_khz() -> None:
    assert _validated_phase_scan_band({}) == (15000.0, 25000.0)
    assert _validated_phase_scan_band(
        {"min_frequency_hz": 16000, "max_frequency_hz": 24000}
    ) == (16000.0, 24000.0)
    with pytest.raises(HTTPException):
        _validated_phase_scan_band(
            {"min_frequency_hz": 14000, "max_frequency_hz": 24000}
        )
    with pytest.raises(HTTPException):
        _validated_phase_scan_band(
            {"min_frequency_hz": 16000, "max_frequency_hz": 26000}
        )
    with pytest.raises(HTTPException):
        _validated_phase_scan_band(
            {"min_frequency_hz": 22000, "max_frequency_hz": 21000}
        )


def test_managed_process_rejects_an_immediate_launch_exit() -> None:
    process = ManagedProcess("early_exit", ["/usr/bin/false"], deque(maxlen=20))
    process.start()
    with pytest.raises(RuntimeError, match="exited during startup"):
        process.require_running_after_startup(timeout_s=0.25)


def test_pinger_terminal_output_closes_complete_launch_group(monkeypatch) -> None:
    class FakeProcess:
        stdout = iter([
            "[pinger_homing_controller] C++ homing state: PROBE -> COMPLETE\n"
        ])

        def wait(self) -> int:
            return 0

    killed: list[tuple[int, signal.Signals]] = []
    monkeypatch.setattr(
        process_manager_module,
        "_kill_process_group",
        lambda pgid, sig: killed.append((pgid, sig)),
    )
    process = ManagedProcess(
        "pinger_homing",
        ["unused"],
        deque(maxlen=20),
        stop_on_output_markers=("-> COMPLETE",),
    )
    process.process = FakeProcess()  # type: ignore[assignment]
    process.pgid = 4321

    process._read_output()

    assert killed == [(4321, signal.SIGINT)]
    assert process._terminal_stop_requested
    assert any("terminal state observed" in line for line in process.log_buffer)
