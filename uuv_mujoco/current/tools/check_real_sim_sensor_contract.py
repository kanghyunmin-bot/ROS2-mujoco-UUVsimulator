#!/usr/bin/env python3
"""Validate the hardware-independent real/sim sensor contract and DVL TF behavior."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_static_tf_cameras import build_camera_frame_specs  # noqa: E402
from bridge.ros2_static_tf_sensors import build_sensor_frame_specs  # noqa: E402
from bridge.ros2_stereo_image import (  # noqa: E402
    DEFAULT_IMAGE_HZ,
    IMX219_CAMERA0_COMPRESSED_TOPIC,
    IMX219_CAMERA0_INFO_TOPIC,
    IMX219_CAMERA0_OPTICAL_FRAME,
    IMX219_CAMERA0_RAW_TOPIC,
    IMX219_CAMERA1_COMPRESSED_TOPIC,
    IMX219_CAMERA1_INFO_TOPIC,
    IMX219_CAMERA1_OPTICAL_FRAME,
    IMX219_CAMERA1_RAW_TOPIC,
)
from bridge.ros2_tf_geometry import quat_identity, quat_x_180  # noqa: E402

CONTRACT_PATH = ROOT / "config" / "real_sim_sensor_contract.json"
REPO_ROOT = ROOT.parents[1]
SCENE_PATHS = (
    ROOT / "scenes" / "tank_current_scene.xml",
    ROOT / "scenes" / "tank_current_scene_cable_test.xml",
)
REAL_ROBOT_LAUNCH_PATH = ROOT.parents[1] / "rospkg" / "src" / "kmu26_auv" / "launch" / "rov_start.launch.py"
REAL_DVL_WRAPPER_PATH = (
    ROOT.parents[1]
    / "rospkg"
    / "src"
    / "kmu26_auv"
    / "launch"
    / "dvl_a50_driver.launch.py"
)
REAL_ROBOT_PACKAGE_PATH = ROOT.parents[1] / "rospkg" / "src" / "kmu26_auv" / "package.xml"
REAL_ROBOT_REPOS_PATH = ROOT.parents[1] / "rospkg" / "real_robot.repos"
WEB_GUI_PATH = ROOT.parents[1] / "rospkg" / "src" / "kmu26_auv_web_gui"

EXPECTED_TRANSFORMS = {
    ("base_link", "fcu_link"): ([0.13135, 0.0, 0.08541], [0.0, 0.0, 0.0]),
    ("fcu_link", "imu_link"): ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ("base_link", "dvl_link"): (
        [-0.00488, 0.0, -0.03910],
        [math.pi, 0.0, 0.0],
    ),
    ("base_link", "depth_link"): (
        [-0.17364, -0.03034, 0.0536],
        [0.0, 0.0, 0.0],
    ),
}

EXPECTED_ENDPOINTS = {
    "/mavros/imu/data": ("sensor_msgs/msg/Imu", "fcu_link", 50.0),
    "/mavros/imu/data_raw": ("sensor_msgs/msg/Imu", "fcu_link", 50.0),
    "/mavros/imu/static_pressure": (
        "sensor_msgs/msg/FluidPressure",
        "fcu_link",
        10.0,
    ),
    "/dvl/data": ("auv_dvl_a50_msg/msg/DVL", "dvl_link", 10.0),
    "/dvl/position": ("auv_dvl_a50_msg/msg/DVLDR", "dvl_link", 5.0),
    "/dvl/twist": (
        "geometry_msgs/msg/TwistWithCovarianceStamped",
        "dvl_link",
        10.0,
    ),
    "/dvl/odometry": (
        "nav_msgs/msg/Odometry",
        "dvl_odom",
        5.0,
    ),
    "/depth/pose": (
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        "odom",
        10.0,
    ),
    "/imx219/camera0/image_raw": (
        "sensor_msgs/msg/Image",
        "imx219_camera0_optical_frame",
        30.0,
    ),
    "/imx219/camera0/image_raw/compressed": (
        "sensor_msgs/msg/CompressedImage",
        "imx219_camera0_optical_frame",
        30.0,
    ),
    "/imx219/camera1/image_raw": (
        "sensor_msgs/msg/Image",
        "imx219_camera1_optical_frame",
        30.0,
    ),
    "/imx219/camera1/image_raw/compressed": (
        "sensor_msgs/msg/CompressedImage",
        "imx219_camera1_optical_frame",
        30.0,
    ),
    "/audio_stamped": (
        "audio_common_msgs/msg/AudioDataStamped",
        "hydrophone_link",
        None,
    ),
    "/mavros/battery": ("sensor_msgs/msg/BatteryState", None, None),
    "/ping360/scan_echo": (
        "ping360_sonar_msgs/msg/SonarEcho",
        "ping360_link",
        None,
    ),
}

OFFICIAL_DVL_URLS = {
    "datasheet": "https://waterlinked.com/datasheets/dvl-a50",
    "tcp_api": "https://docs.waterlinked.com/dvl/dvl-json-protocol/",
    "integration": "https://docs.waterlinked.com/dvl/integration/",
}

LOCAL_BAG_SUMMARY = (
    "document/docsource/bar30_z_check_20260504/"
    "bag_2026-04-01_20-08-11/summary.json"
)

DVL_RATE_EVIDENCE = {
    "/dvl/data": {
        "range_hz": [4.0, 15.0],
        "observed_hz": 9.275646435125926,
        "rate_source": OFFICIAL_DVL_URLS["datasheet"],
        "observed_source": LOCAL_BAG_SUMMARY,
    },
    "/dvl/position": {
        "range_hz": [5.0, 5.0],
        "observed_hz": 4.413036175468005,
        "rate_source": OFFICIAL_DVL_URLS["tcp_api"],
        "observed_source": LOCAL_BAG_SUMMARY,
    },
    "/dvl/twist": {
        "range_hz": [4.0, 15.0],
        "observed_hz": 8.946948401873865,
        "rate_source": OFFICIAL_DVL_URLS["datasheet"],
        "observed_source": LOCAL_BAG_SUMMARY,
    },
    "/dvl/odometry": {
        "range_hz": [5.0, 5.0],
        "observed_hz": None,
        "rate_source": OFFICIAL_DVL_URLS["tcp_api"],
        "observed_source": (
            "not_recorded_in_bag_2026-04-01_20-08-11; upstream /dvl/position was "
            "4.413036175468005 Hz"
        ),
    },
}


def load_contract() -> dict:
    payload = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    assert isinstance(payload, dict)
    return payload


def test_schema_and_time_contract(payload: dict) -> None:
    assert payload["schema_version"] == 1
    assert payload["contract_id"] == "kmu_auv_real_sim_sensor_contract_v1"
    time_contract = payload["time_contract"]
    assert time_contract["clock_topic"] == "/clock"
    assert time_contract["simulation_uses_ros_time"] is True
    assert time_contract["a50_device_mode_driver_use_sim_time"] is True
    assert time_contract["authoritative_clock_survives_optional_ros_topic_failure"] is True
    assert time_contract["header_stamp_policy"] == {
        "native_sim_sensor_topics": "capture_time",
        "mavros_imu_data_strict_sim": (
            "external_ardusub_mavros_ahrs_message_time"
        ),
        "mavros_imu_data_raw_strict_sim": "modeled_capture_time",
        "mavros_imu_static_pressure_strict_sim": "modeled_capture_time",
        "imx219_physical_driver_topics": (
            "gstreamer_pts_mapped_to_ros_clock_with_strictly_monotonic_"
            "frame_period_fallback"
        ),
        "imx219_sim_alias_topics": "mujoco_render_snapshot_capture_time",
        "a50_physical_driver_topics": "host_receive_time_from_driver_Node_now",
        "a50_direct_ros_diagnostic_mode": (
            "host_receive_time_matching_physical_driver"
        ),
    }
    assert time_contract["dvl_slam_measurement_time_source"] == (
        "time_of_validity_after_device_host_clock_alignment"
    )
    assert time_contract["arrival_time_must_not_replace_dvl_payload_capture_fields"] is True

    real_launch_source = REAL_ROBOT_LAUNCH_PATH.read_text(encoding="utf-8")
    wrapper_source = REAL_DVL_WRAPPER_PATH.read_text(encoding="utf-8")
    package_source = REAL_ROBOT_PACKAGE_PATH.read_text(encoding="utf-8")
    assert '"dvl_a50_driver.launch.py"' in real_launch_source
    assert '"use_sim_time": use_sim_time' in real_launch_source
    assert '"use_sim_time": ParameterValue(' in wrapper_source
    assert '<exec_depend>auv_dvl_a50</exec_depend>' in package_source

    provenance = payload["provenance"]
    assert provenance["dvl_a50_datasheet"] == OFFICIAL_DVL_URLS["datasheet"]
    assert provenance["dvl_a50_tcp_json_api"] == OFFICIAL_DVL_URLS["tcp_api"]
    assert provenance["dvl_integration_documentation"] == OFFICIAL_DVL_URLS["integration"]
    assert provenance["imx219_repository"] == (
        "https://github.com/2026-kmu-underwater-robot/auv_imx219_camera"
    )
    assert provenance["imx219_commit"] == "a6b5b9455f082c201496326bfdaee2791f4a4d90"
    assert provenance["local_bag_rate_summary"] == LOCAL_BAG_SUMMARY


def test_pinned_dvl_dependencies(payload: dict) -> None:
    provenance = payload["provenance"]
    repos_source = REAL_ROBOT_REPOS_PATH.read_text(encoding="utf-8")
    expected_entries = (
        "  auv_dvl_a50:\n"
        "    type: git\n"
        f"    url: {provenance['dvl_repository']}.git\n"
        f"    version: {provenance['dvl_commit']}",
        "  auv_dvl_a50_msg:\n"
        "    type: git\n"
        f"    url: {provenance['dvl_message_repository']}.git\n"
        f"    version: {provenance['dvl_message_commit']}",
        "  auv_imx219_camera:\n"
        "    type: git\n"
        f"    url: {provenance['imx219_repository']}.git\n"
        f"    version: {provenance['imx219_commit']}",
    )
    for entry in expected_entries:
        assert entry in repos_source, "real_robot.repos does not match DVL provenance"


def test_web_gui_uses_single_physical_dvl_message_type() -> None:
    interface_source = (
        WEB_GUI_PATH / "kmu26_auv_web_gui" / "ros_interface.py"
    ).read_text(encoding="utf-8")
    package_source = (WEB_GUI_PATH / "package.xml").read_text(encoding="utf-8")

    assert "from auv_dvl_a50_msg.msg import DVL" in interface_source
    assert "from auv_dvl_a50_msg.msg import CommandResponse" in interface_source
    assert "from auv_dvl_a50_msg.msg import ConfigCommand" in interface_source
    assert "from auv_dvl_a50_msg.msg import ConfigStatus" in interface_source
    assert "from dvl_msgs.msg" not in interface_source
    assert interface_source.count(
        'create_subscription(DVL, "/dvl/data", self._on_dvl_data, sensor_qos)'
    ) == 1
    assert "<exec_depend>auv_dvl_a50_msg</exec_depend>" in package_source
    assert "<exec_depend>dvl_msgs</exec_depend>" not in package_source


def test_transform_contract(payload: dict) -> None:
    transforms = payload["frames"]["static_transforms"]
    indexed = {(item["parent"], item["child"]): item for item in transforms}
    assert len(indexed) == len(transforms), "duplicate static TF parent/child pair"
    children = [item["child"] for item in transforms]
    assert len(children) == len(set(children)), "static TF child has multiple parents"
    assert set(EXPECTED_TRANSFORMS).issubset(indexed)
    for key, (translation, rotation) in EXPECTED_TRANSFORMS.items():
        item = indexed[key]
        np.testing.assert_allclose(item["translation_m"], translation, atol=1e-12)
        np.testing.assert_allclose(item["rotation_rpy_rad"], rotation, atol=1e-12)
        assert item["status"] == "public_real_default"


def test_endpoint_contract(payload: dict) -> None:
    endpoints = payload["endpoints"]
    ids = [item["id"] for item in endpoints]
    topics = [item["topic"] for item in endpoints]
    assert len(ids) == len(set(ids)), "duplicate endpoint id"
    assert len(topics) == len(set(topics)), "duplicate endpoint topic"

    known_frames = set(payload["frames"]["world"] + payload["frames"]["vehicle"])
    indexed = {item["topic"]: item for item in endpoints}
    assert set(EXPECTED_ENDPOINTS).issubset(indexed)
    for endpoint in endpoints:
        assert endpoint["topic"].startswith("/")
        assert "/msg/" in endpoint["type"]
        assert endpoint["rate_basis"]
        if endpoint["nominal_rate_hz"] is not None:
            assert endpoint["nominal_rate_hz"] > 0.0
        if endpoint["frame"] is not None:
            assert endpoint["frame"] in known_frames
        if "child_frame" in endpoint:
            assert endpoint["child_frame"] in known_frames

    for topic, (message_type, frame, rate_hz) in EXPECTED_ENDPOINTS.items():
        endpoint = indexed[topic]
        assert endpoint["type"] == message_type
        assert endpoint["frame"] == frame
        assert endpoint["nominal_rate_hz"] == rate_hz

    for topic, evidence in DVL_RATE_EVIDENCE.items():
        endpoint = indexed[topic]
        assert endpoint["supported_rate_range_hz"] == evidence["range_hz"]
        assert endpoint["rate_range_basis"]
        assert endpoint["observed_rate_hz"] == evidence["observed_hz"]
        assert endpoint["rate_source"] == evidence["rate_source"]
        assert endpoint["observed_rate_source"] == evidence["observed_source"]

    assert indexed["/dvl/data"]["qos_profile"] == "sensor_data_best_effort"
    assert indexed["/dvl/position"]["qos_profile"] == "sensor_data_best_effort"
    assert indexed["/dvl/data"]["timestamp"] == (
        "driver_receive_time; device capture is time_of_validity"
    )
    assert indexed["/dvl/position"]["timestamp"] == (
        "driver_receive_time; device report time is message.time"
    )
    fused_imu = indexed["/mavros/imu/data"]
    assert fused_imu["timestamp"] == "external_ardusub_mavros_ahrs_message_time"
    assert fused_imu["layer"] == "fcu_ahrs"
    assert fused_imu["sim_owner"] == "external_ardusub_mavros"
    assert fused_imu["orientation_source"] == "ardusub_ahrs"
    raw_imu = indexed["/mavros/imu/data_raw"]
    assert raw_imu["timestamp"] == "capture_time"
    assert raw_imu["layer"] == "raw_sensor"
    assert raw_imu["sim_owner"] == "uuv_mujoco_delivery_bridge"
    assert raw_imu["measurement_source"] == (
        "modeled_accelerometer_and_gyroscope"
    )
    assert raw_imu["orientation_semantics"] == {
        "value_wxyz": [1.0, 0.0, 0.0, 0.0],
        "covariance": [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "meaning": "orientation_not_provided",
    }
    pressure = indexed["/mavros/imu/static_pressure"]
    assert pressure["timestamp"] == "capture_time"
    assert pressure["sim_owner"] == "uuv_mujoco_delivery_bridge"

    gt = indexed["/mujoco/ground_truth/pose"]
    assert gt["frame"] == "world"
    assert gt["layer"] == "evaluation_only"
    assert gt["required_for_slam_mvp"] is False
    assert {"world", "map", "odom", "dvl_odom"}.issubset(
        set(payload["frames"]["world"])
    )
    ground_truth_factory = (
        ROOT / "bridge" / "ros2_publish_core_factories.py"
    ).read_text(encoding="utf-8")
    assert 'build_pose_msg(bridge.PoseStamped, stamp, "world"' in ground_truth_factory

    safety = payload["safety_invariants"]
    oracles = safety["simulation_only_ground_truth_oracles"]
    assert set(oracles) == {"/mujoco/ground_truth/pose", "/sim/odom"}
    assert "forbidden" in oracles["/sim/odom"]
    filtered_policy = safety["standard_estimator_topic_ownership"][
        "/odometry/filtered"
    ]
    assert filtered_policy["bridge_default_publisher"] is False
    assert filtered_policy["required_owner"] == "external_estimator"
    assert filtered_policy["unsafe_legacy_cli_opt_in"] == (
        "--unsafe-legacy-ground-truth-odometry-filtered"
    )
    assert filtered_policy["unsafe_legacy_environment_opt_in"] == (
        "ROS2_UUV_UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED=1"
    )
    assert filtered_policy["strict_real_pkg_compat_policy"] == "forbidden"


def test_device_boundaries(payload: dict) -> None:
    devices = payload["device_boundaries"]
    assert devices["fcu"]["real_default"] == "/dev/ttyACM0:57600"
    assert devices["fcu"]["sim_plant_transport"] == (
        "mandatory_400_hz_json_sample_hold_for_fcu_controller_only"
    )
    assert devices["fcu"]["strict_sim_raw_topic_provider"] == (
        "mujoco_bridge_delivery_driven_sensor_model"
    )
    assert devices["fcu"]["strict_sim_public_raw_topics"] == [
        "/mavros/imu/data_raw",
        "/mavros/imu/static_pressure",
    ]
    assert devices["fcu"]["strict_sim_ahrs_topic_provider"] == (
        "external_ardusub_mavros"
    )
    assert devices["fcu"]["strict_sim_public_ahrs_topics"] == [
        "/mavros/imu/data"
    ]
    assert devices["fcu"]["strict_sim_mavros_passthrough_namespace"] == (
        "/uuv_mujoco/mavros_fcu_passthrough"
    )
    assert devices["dvl"]["model"] == "Water Linked A50"
    assert devices["dvl"]["real_default_host"] == "192.168.194.95"
    assert devices["dvl"]["real_default_port"] == 16171
    dvl = devices["dvl"]
    assert dvl["driver_package"] == "auv_dvl_a50"
    assert dvl["message_package"] == "auv_dvl_a50_msg"
    assert dvl["default_sim_mode"] == {
        "strict_real_pkg_compat": "tcp_driver",
        "non_strict": "direct_ros",
    }
    assert dvl["recommended_sim_to_real_mode"] == "tcp_driver"
    assert dvl["tcp_driver_enable_environment"] == (
        "ROS2_UUV_DVL_DEVICE_EMULATOR_ENABLE=1"
    )
    assert dvl["strict_direct_ros_opt_out_allowed"] is False
    operating_spec = dvl["official_operating_specification"]
    assert operating_spec["transducer_beam_angle_deg"] == 22.5
    assert operating_spec["minimum_altitude_m"] == 0.05
    assert operating_spec["maximum_altitude_m"] == 50.0
    assert operating_spec["adaptive_ping_rate_range_hz"] == [4.0, 15.0]
    assert operating_spec["source"] == OFFICIAL_DVL_URLS["datasheet"]

    tcp_contract = dvl["official_tcp_report_contract"]
    assert tcp_contract["velocity_report_rate"] == "adaptive_to_altitude_and_configuration"
    assert tcp_contract["dead_reckoning_expected_rate_hz"] == 5.0
    assert tcp_contract["source"] == OFFICIAL_DVL_URLS["tcp_api"]
    latency = tcp_contract["network_latency_ms"]
    assert latency == {
        "mean": 4.0,
        "standard_deviation": 2.0,
        "observed_maximum": 13.0,
        "hard_realtime_guarantee": False,
    }

    local_rates = dvl["local_rate_observation"]
    assert local_rates["source"] == LOCAL_BAG_SUMMARY
    assert local_rates["dvl_data_hz"] == DVL_RATE_EVIDENCE["/dvl/data"]["observed_hz"]
    assert local_rates["dvl_position_hz"] == DVL_RATE_EVIDENCE["/dvl/position"]["observed_hz"]
    assert local_rates["dvl_twist_hz"] == DVL_RATE_EVIDENCE["/dvl/twist"]["observed_hz"]

    noise_policy = dvl["noise_parameter_policy"]
    assert noise_policy["classification"] == "pre_bag_calibration_prior"
    assert noise_policy["is_official_manufacturer_specification"] is False
    assert noise_policy["must_be_fitted_and_validated_with_real_bag_data"] is True
    assert devices["hydrophone"]["sample_rate_hz"] == 96000
    assert devices["hydrophone"]["channels"] == 2
    assert devices["hydrophone"]["sample_format"] == "S32LE"
    imx219 = devices["imx219"]
    assert imx219["driver_package"] == "auv_imx219_camera"
    assert imx219["default_resolution"] == [1280, 720]
    assert imx219["default_rate_hz"] == 30.0
    assert imx219["raw_encoding"] == "bgr8"
    assert imx219["compressed_format"] == "bgr8; jpeg compressed bgr8"
    assert imx219["timestamp_source"] == "gstreamer_pts"


def test_local_bag_rate_evidence() -> None:
    summary_path = REPO_ROOT / LOCAL_BAG_SUMMARY
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    rates = summary["rates"]
    for topic in ("/dvl/data", "/dvl/position", "/dvl/twist"):
        measured = rates[topic]["mean_rate_hz_over_bag"]
        expected = DVL_RATE_EVIDENCE[topic]["observed_hz"]
        assert measured == expected


def test_dvl_site_quaternion_is_preserved() -> None:
    positions = np.array(
        [
            [0.11, 0.0, 0.09],
            [-0.17, -0.03, 0.05],
            [-0.01, 0.02, -0.04],
            [0.0, 0.0, 0.2],
            [0.2, 0.0, -0.03],
        ],
        dtype=np.float64,
    )
    dvl_quat = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)
    quaternions = np.array(
        [quat_identity(), quat_identity(), dvl_quat, quat_identity(), quat_identity()],
        dtype=np.float64,
    )
    model = SimpleNamespace(site_pos=positions, site_quat=quaternions)
    specs = build_sensor_frame_specs(
        model=model,
        imu_site_id=0,
        bar30_site_id=1,
        dvl_site_id=2,
        ping360_site_id=3,
        hydrophone_site_id=4,
        ping360_frame_id="ping360_link",
        zero=np.zeros(3, dtype=np.float64),
        ident=quat_identity(),
        x180=quat_x_180(),
    )
    indexed = {(parent, child): (position, quat) for parent, child, position, quat in specs}
    np.testing.assert_allclose(indexed[("base_link", "dvl_link")][0], positions[2])
    np.testing.assert_allclose(indexed[("base_link", "dvl_link")][1], dvl_quat)
    np.testing.assert_allclose(indexed[("dvl_link", "dvl")][1], quat_identity())


def test_public_real_fallback_transforms() -> None:
    model = SimpleNamespace(
        site_pos=np.empty((0, 3), dtype=np.float64),
        site_quat=np.empty((0, 4), dtype=np.float64),
    )
    specs = build_sensor_frame_specs(
        model=model,
        imu_site_id=-1,
        bar30_site_id=-1,
        dvl_site_id=-1,
        ping360_site_id=-1,
        hydrophone_site_id=-1,
        ping360_frame_id="ping360_link",
        zero=np.zeros(3, dtype=np.float64),
        ident=quat_identity(),
        x180=quat_x_180(),
    )
    indexed = {(parent, child): (position, quat) for parent, child, position, quat in specs}
    np.testing.assert_allclose(
        indexed[("base_link", "fcu_link")][0],
        EXPECTED_TRANSFORMS[("base_link", "fcu_link")][0],
    )
    np.testing.assert_allclose(
        indexed[("base_link", "depth_link")][0],
        EXPECTED_TRANSFORMS[("base_link", "depth_link")][0],
    )
    np.testing.assert_allclose(
        indexed[("base_link", "dvl_link")][0],
        EXPECTED_TRANSFORMS[("base_link", "dvl_link")][0],
    )
    np.testing.assert_allclose(indexed[("base_link", "dvl_link")][1], quat_x_180())


def test_imx219_runtime_contract() -> None:
    assert DEFAULT_IMAGE_HZ == 30.0
    assert IMX219_CAMERA0_RAW_TOPIC == "/imx219/camera0/image_raw"
    assert IMX219_CAMERA0_COMPRESSED_TOPIC == "/imx219/camera0/image_raw/compressed"
    assert IMX219_CAMERA0_INFO_TOPIC == "/imx219/camera0/camera_info"
    assert IMX219_CAMERA1_RAW_TOPIC == "/imx219/camera1/image_raw"
    assert IMX219_CAMERA1_COMPRESSED_TOPIC == "/imx219/camera1/image_raw/compressed"
    assert IMX219_CAMERA1_INFO_TOPIC == "/imx219/camera1/camera_info"

    specs = build_camera_frame_specs(
        model=SimpleNamespace(),
        cam_left_site_id=-1,
        cam_right_site_id=-1,
        zero=np.zeros(3, dtype=np.float64),
        ident=quat_identity(),
        optical_quat=np.array([0.5, -0.5, 0.5, -0.5], dtype=np.float64),
    )
    indexed = {(parent, child): (position, quat) for parent, child, position, quat in specs}
    assert ("stereo_left", IMX219_CAMERA0_OPTICAL_FRAME) in indexed
    assert ("stereo_right", IMX219_CAMERA1_OPTICAL_FRAME) in indexed


def test_active_scene_sensor_transforms() -> None:
    expected_sites = {
        "imu_site": ([0.13135, 0.0, 0.08541], [1.0, 0.0, 0.0, 0.0]),
        "bar30_site": ([-0.17364, -0.03034, 0.0536], [1.0, 0.0, 0.0, 0.0]),
        "dvl_site": ([-0.00488, 0.0, -0.03910], [0.0, 1.0, 0.0, 0.0]),
    }
    for scene_path in SCENE_PATHS:
        root = ET.parse(scene_path).getroot()
        for site_name, (expected_position, expected_quaternion) in expected_sites.items():
            site = root.find(f".//site[@name='{site_name}']")
            assert site is not None, f"{scene_path.name}: missing {site_name}"
            position = np.fromstring(site.attrib["pos"], sep=" ")
            quaternion = np.fromstring(site.attrib["quat"], sep=" ")
            np.testing.assert_allclose(position, expected_position, atol=1e-12)
            np.testing.assert_allclose(quaternion, expected_quaternion, atol=1e-12)


def test_real_robot_launch_uses_contract_defaults() -> None:
    source = REAL_ROBOT_LAUNCH_PATH.read_text(encoding="utf-8")
    required_defaults = (
        'DeclareLaunchArgument("dvl_frame", default_value="dvl_link")',
        'DeclareLaunchArgument("base_to_fcu_x", default_value="0.13135")',
        'DeclareLaunchArgument("base_to_fcu_y", default_value="0.0")',
        'DeclareLaunchArgument("base_to_fcu_z", default_value="0.08541")',
        'DeclareLaunchArgument("dvl_x", default_value="-0.00488")',
        'DeclareLaunchArgument("dvl_z", default_value="-0.03910")',
        'DeclareLaunchArgument("dvl_roll", default_value="3.141592653589793")',
        'DeclareLaunchArgument("dvl_position_child_frame", default_value="dvl_link")',
        'os.path.join(package_share, "launch", "dvl_a50_driver.launch.py")',
        '"auv_imx219_camera", os.path.join("launch", "dual_imx219.launch.py")',
        'DeclareLaunchArgument("use_imx219", default_value="true")',
        'DeclareLaunchArgument("imx219_timestamp_source", default_value="gstreamer_pts")',
        '"timestamp_source": imx219_timestamp_source',
        '"use_sim_time": use_sim_time',
    )
    for declaration in required_defaults:
        assert declaration in source, f"real launch missing contract default: {declaration}"

    package_xml = REAL_ROBOT_LAUNCH_PATH.parents[1] / "package.xml"
    package_source = package_xml.read_text(encoding="utf-8")
    assert "<depend>auv_dvl_a50_msg</depend>" in package_source
    assert "<exec_depend>auv_dvl_a50</exec_depend>" in package_source
    assert "<exec_depend>auv_imx219_camera</exec_depend>" in package_source


def main() -> int:
    payload = load_contract()
    test_schema_and_time_contract(payload)
    test_pinned_dvl_dependencies(payload)
    test_web_gui_uses_single_physical_dvl_message_type()
    test_transform_contract(payload)
    test_endpoint_contract(payload)
    test_device_boundaries(payload)
    test_local_bag_rate_evidence()
    test_dvl_site_quaternion_is_preserved()
    test_public_real_fallback_transforms()
    test_imx219_runtime_contract()
    test_active_scene_sensor_transforms()
    test_real_robot_launch_uses_contract_defaults()
    print(
        "real_sim_sensor_contract=PASS "
        f"endpoints={len(payload['endpoints'])} "
        f"static_transforms={len(payload['frames']['static_transforms'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
