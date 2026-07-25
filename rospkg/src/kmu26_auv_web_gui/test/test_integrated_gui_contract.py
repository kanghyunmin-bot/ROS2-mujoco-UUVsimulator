from copy import deepcopy
from html.parser import HTMLParser
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from kmu26_auv_web_gui.ros_interface import LocalizationRosNode
from kmu26_auv_web_gui.ros_interface import RosInterface
from kmu26_auv_web_gui.ros_interface import COMPRESSED_IMAGE_TYPE
from kmu26_auv_web_gui.ros_interface import RAW_IMAGE_TYPE
from kmu26_auv_web_gui.ros_interface import _vision_image_topic_options
from kmu26_auv_web_gui.server import _pinger_live_preflight
from kmu26_auv_web_gui.server import VISION_MISSION_LAUNCH_ARGS
from kmu26_auv_web_gui.server import VISION_YOLO_LAUNCH_ARGS


ROOT = Path(__file__).resolve().parents[1]
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

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
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


class _VisionLaunchInputParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.arguments = {"yolo": set(), "mission": set()}
        self.defaults = {"yolo": {}, "mission": {}}

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        if tag not in {"input", "select"}:
            return
        values = dict(attrs)
        for group in ("yolo", "mission"):
            name = values.get(f"data-vision-{group}")
            if not name:
                continue
            self.arguments[group].add(name)
            if tag == "input":
                is_checkbox = values.get("type") == "checkbox"
                default = (
                    "true"
                    if is_checkbox and "checked" in values
                    else values.get("value", "")
                )
                if is_checkbox and "checked" not in values:
                    default = "false"
                self.defaults[group][name] = default


def test_vision_and_pinger_tabs_are_siblings_and_script_is_loaded_once() -> None:
    parser = _LayoutParser()
    parser.feed((ROOT / "web" / "index.html").read_text(encoding="utf-8"))

    assert parser.section_parents["vision-tab"] == ""
    assert parser.section_parents["pinger-tab"] == ""
    assert len(parser.app_scripts) == 1
    assert len(parser.ids) == len(set(parser.ids))


def test_vision_image_topic_selector_contract_is_complete() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
    javascript = (ROOT / "web" / "app.js").read_text(encoding="utf-8")

    frame_shell = html.index('<div class="vision-frame-shell">')
    selector = html.index('id="vision-frame-topic"', frame_shell)
    canvas_wrap = html.index('<div class="vision-canvas-wrap">', frame_shell)
    canvas = html.index('id="vision-canvas"', canvas_wrap)
    assert frame_shell < selector < canvas_wrap < canvas
    assert 'id="vision-feed-empty-topic"' in html
    assert html.count("?v=20260719-buoy-launch-sync") == 2
    assert ".vision-frame-source" in css
    assert "grid-template-columns: minmax(0, 7fr) minmax(320px, 3fr);" in css
    assert css.count("{") == css.count("}")
    assert 'getJson("/api/vision/image_topics")' in javascript
    assert 'postJson("/api/vision/image_source"' in javascript
    assert 'response.headers.get("X-Vision-Frame-Topic")' in javascript
    assert "frameSourceGeneration" in javascript


def test_vision_launch_configuration_is_collapsed_by_default() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")

    disclosure = '<details class="vision-config-disclosure">'
    disclosure_start = html.index(disclosure)
    summary = html.index("<summary>", disclosure_start)
    content = html.index('<div class="vision-config-content">', summary)
    assert "open" not in disclosure
    assert disclosure_start < summary < content
    assert 'class="vision-config-chevron" aria-hidden="true"' in html
    assert ".vision-config-disclosure[open]" in css


def test_vision_launch_inputs_match_package_launch_contract() -> None:
    parser = _VisionLaunchInputParser()
    parser.feed((ROOT / "web" / "index.html").read_text(encoding="utf-8"))

    assert parser.arguments["yolo"] == VISION_YOLO_LAUNCH_ARGS
    assert parser.arguments["mission"] == VISION_MISSION_LAUNCH_ARGS
    assert parser.defaults["mission"] == {
        "bbox_topic": "/vision/buoy_bbox",
        "depth_topic": "/auv/depth",
        "depth_pose_topic": "/depth/pose",
        "depth_pose_scale": "-1.0",
        "depth_pose_offset_m": "0.0",
        "enable_topic": "/mission/control_enable",
        "state_topic": "/mission/state",
        "rc_override_topic": "/mavros/rc/override",
        "rc_monitor_topic": "/mission/rc_command",
        "control_rate_hz": "20",
        "throttle_channel": "3",
        "yaw_channel": "4",
        "forward_channel": "5",
        "neutral_pwm": "1500",
        "min_pwm": "1300",
        "max_pwm": "1700",
        "max_yaw_delta": "180",
        "forward_pwm": "1700",
        "approach_forward_min_pwm": "1560",
        "search_yaw_pwm": "1600",
        "yaw_invert": "false",
        "vertical_positive_is_up": "true",
        "work_depth_m": "0.4",
        "surface_depth_m": "0.1",
        "max_depth_m": "1.5",
        "buoyancy_hold_delta_pwm": "40",
        "lpf_tau_sec": "0.3",
        "buoy_class_id": "0",
        "stick_class_id": "1",
        "min_detection_hits": "5",
        "approach_area_ratio": "0.30",
        "approach_vision_throttle_weight": "0.4",
        "fork_target_x": "0.30",
        "fork_target_y": "0.70",
        "stick_deadband_x": "0.06",
        "stick_deadband_y": "0.08",
        "align_stable_sec": "0.7",
        "insert_pwm": "1560",
        "insert_duration_sec": "0.8",
        "detach_pwm": "1620",
        "detach_duration_sec": "0.3",
        "backoff_pwm": "1420",
        "backoff_duration_sec": "0.5",
        "search_timeout_sec": "40.0",
        "area_verify_sec": "12.0",
    }


def test_vision_rc_channels_use_compact_status_grid() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
    javascript = (ROOT / "web" / "app.js").read_text(encoding="utf-8")

    assert 'class="vision-rc-grid" role="list"' in html
    assert 'class="vision-rc-legend"' in html
    assert "grid-template-columns: repeat(6, minmax(0, 1fr));" in css
    assert 'displayValue = "REL"' in javascript
    assert 'displayValue = "N/C"' in javascript
    assert 'shortLabel: "V"' in javascript
    state_panel = html.index('<article class="panel vision-state-panel">')
    rc_grid = html.index('id="vision-rc-grid"', state_panel)
    detections = html.index('id="vision-detections"', rc_grid)
    assert state_panel < rc_grid < detections


def test_vision_image_topic_options_only_include_supported_types() -> None:
    options = _vision_image_topic_options(
        {
            "/camera/raw": [RAW_IMAGE_TYPE],
            "/camera/compressed": [COMPRESSED_IMAGE_TYPE],
            "/camera/info": ["sensor_msgs/msg/CameraInfo"],
        },
        "/vision/yolo/annotated/compressed",
        COMPRESSED_IMAGE_TYPE,
    )

    assert [item["topic"] for item in options] == [
        "/camera/compressed",
        "/camera/raw",
        "/vision/yolo/annotated/compressed",
    ]
    assert options[-1]["available"] is False


def test_vision_frame_source_switches_to_raw_image_topic() -> None:
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init(args=[])
    node = LocalizationRosNode()
    try:
        node.create_publisher(
            Image,
            "/test/gui/raw_image",
            qos_profile_sensor_data,
        )
        selected = node.select_vision_frame_topic("/test/gui/raw_image")
        assert selected == {
            "topic": "/test/gui/raw_image",
            "type": RAW_IMAGE_TYPE,
        }

        msg = Image()
        msg.height = 2
        msg.width = 2
        msg.encoding = "rgb8"
        msg.step = 6
        msg.data = bytes([255, 0, 0] * 4)
        node._on_vision_raw_image(
            msg,
            selected["topic"],
            node._vision_subscription_generation,
        )

        deadline = time.monotonic() + 2.0
        frame = node.latest_vision_frame()
        while not frame[0] and time.monotonic() < deadline:
            time.sleep(0.01)
            frame = node.latest_vision_frame()
        data, content_type, sequence, topic = frame
        assert data.startswith(b"\xff\xd8")
        assert content_type == "image/jpeg"
        assert sequence == 1
        assert topic == selected["topic"]
    finally:
        node.destroy_node()
        if owned_context and rclpy.ok():
            rclpy.shutdown()


def test_vision_frame_source_task_contains_selection_errors() -> None:
    class InvalidSelectionNode:
        @staticmethod
        def select_vision_frame_topic(topic: str) -> dict:
            raise ValueError(f"invalid image topic: {topic}")

    result = RosInterface._select_vision_frame_topic_task(
        InvalidSelectionNode(),
        "/not/an/image",
    )
    assert result["selected"] == {}
    assert result["invalid"] is True
    assert result["error"] == "invalid image topic: /not/an/image"


def test_pinger_parameter_controls_and_css_contract_are_complete() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
    javascript = (ROOT / "web" / "app.js").read_text(encoding="utf-8")

    assert html.count("data-pinger-param") == 13
    assert 'id="pinger-parameter-reset"' in html
    assert 'id="pinger-parameter-summary"' in html
    assert css.count("{") == css.count("}")
    assert ".vision-log-output" in css
    assert ".pinger-parameter-groups" in css
    assert "function validatePingerParameters" in javascript
    assert "bindPingerParameterControls();" in javascript


def test_pinger_top_view_contract_is_complete() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
    javascript = (ROOT / "web" / "app.js").read_text(encoding="utf-8")

    assert 'id="pinger-top-view"' in html
    assert 'id="pinger-top-view-status"' in html
    assert ".pinger-top-view" in css
    assert "function renderPingerTopView" in javascript
    assert "pinger.dry_run ? pinger.requested_command : pinger.command" in javascript
    assert "renderPingerTopView(ros);" in javascript


def test_dvl_calibration_ui_waits_for_ack_contract() -> None:
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
    javascript = (ROOT / "web" / "app.js").read_text(encoding="utf-8")

    assert 'id="dvl-calibrate"' in html
    assert 'id="dvl-calibration-state"' in html
    assert 'aria-live="polite"' in html
    assert "function startDvlCalibration" in javascript
    assert "function renderDvlCalibration" in javascript
    assert 'completed: `COMPLETE · ACK ${calibration.completed_at || "received"}`' in javascript
    assert 'event.type === "sent"' in javascript
    assert ".dvl-calibration-status.completed" in css


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
        assert "/mavros/rc/override" not in publisher_topics
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
    }
    body = {
        "use_hydrophone_estimator": True,
        "use_audio_capture": False,
        "max_runtime_s": 180,
        "arrival_radius_m": 1.5,
    }

    valid = _pinger_live_preflight(status, body)
    assert valid["ok"]
    assert len(valid["checks"]) == 18

    positive_depth = deepcopy(status)
    positive_depth["pose"]["z"] = 1.2
    positive_depth["depth"]["z"] = 1.1
    invalid = _pinger_live_preflight(positive_depth, body)
    depth_check = next(item for item in invalid["checks"] if item["name"] == "depth_sign")
    assert not invalid["ok"]
    assert not depth_check["ok"]
