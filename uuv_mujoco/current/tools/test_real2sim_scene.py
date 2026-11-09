"""Coverage for sensor insertion across modular MJCF includes."""

from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

from real2sim_scene import prepare_scene


def test_prepare_scene_replaces_sensors_from_included_body(tmp_path: Path) -> None:
    model = tmp_path / "input"
    model.mkdir()
    (model / "scene.xml").write_text(
        '<mujoco><worldbody><body name="base_link"><include file="body.xml"/>'
        '</body></worldbody><sensor><gyro name="imu_gyro" site="imu_site"/>'
        '</sensor></mujoco>', encoding="utf-8"
    )
    (model / "body.xml").write_text(
        '<mujocoinclude><site name="imu_site" pos="0 0 0"/>'
        '<camera name="stereo_left" pos="0 0 0"/></mujocoinclude>', encoding="utf-8"
    )
    sensors = {
        "imu": {"pos_m": [1, 2, 3], "quat_wxyz": [2, 0, 0, 0]},
        "dvl": {"pos_m": [0, 0, -1], "quat_wxyz": [0, 1, 0, 0]},
    }
    scene, normalized = prepare_scene(model / "scene.xml", model, sensors, tmp_path / "output")
    root = ET.parse(scene).getroot()
    assert len(root.findall(".//site[@name='imu_site']")) == 1
    assert root.find(".//site[@name='imu_site']").get("pos") == "1 2 3"
    assert root.find(".//camera[@name='stereo_left']") is None
    assert root.find(".//rangefinder[@name='dvl_altitude']").get("site") == "dvl_site"
    assert normalized["imu"]["quat_wxyz"] == [1, 0, 0, 0]
    assert 'pos="0 0 0"' in (model / "body.xml").read_text(encoding="utf-8")


def test_prepare_scene_rejects_include_outside_model(tmp_path: Path) -> None:
    model = tmp_path / "input"
    model.mkdir()
    (tmp_path / "outside.xml").write_text("<mujocoinclude><site/></mujocoinclude>", encoding="utf-8")
    (model / "scene.xml").write_text(
        '<mujoco><worldbody><body name="base_link"><include file="../outside.xml"/>'
        '</body></worldbody></mujoco>', encoding="utf-8"
    )
    with pytest.raises(ValueError, match="outside model directory"):
        prepare_scene(model / "scene.xml", model, {}, tmp_path / "output")


def test_camera_eye_and_mount_can_have_different_transforms(tmp_path: Path) -> None:
    model = tmp_path / "input"
    model.mkdir()
    source = model / "scene.xml"
    source.write_text('<mujoco><worldbody><body name="base_link"/></worldbody></mujoco>', encoding="utf-8")
    scene, _ = prepare_scene(source, model, {"camera_left": {
        "pos_m": [.25, 0, 0], "quat_wxyz": [.5, .5, -.5, -.5],
        "mount_pos_m": [.125, 0, 0], "mount_quat_wxyz": [1, 0, 0, 0],
    }}, tmp_path / "output")
    root = ET.parse(scene)
    assert root.find(".//site[@name='cam_left_site']").get("pos") == "0.125 0 0"
    assert root.find(".//site[@name='cam_left_site']").get("quat") == "1 0 0 0"
    assert root.find(".//camera[@name='stereo_left']").get("pos") == "0.25 0 0"
