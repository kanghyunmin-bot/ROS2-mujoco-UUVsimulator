"""Verify launch selections reach real sensor loaders with auditable provenance."""

import hashlib
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gui.sensor_error_mode import configure_sensor_error_mode, resolve_sensor_error_mode
from bridge.ros2_imu_bar30_sensor_runtime import _load_profile
from bridge.ros2_dvl_sensor_runtime import _load_sensor_config


def test_existing_preserves_explicit_overrides():
    original = {"ROS2_UUV_IMU_SENSOR_ENABLE": "0", "ROS2_UUV_DVL_SENSOR_SEED": "17"}
    env, snapshot = configure_sensor_error_mode(original, None)
    assert env == original
    assert env is not original
    assert snapshot["mode"] == "existing"
    with patch.dict(os.environ, env, clear=True):
        assert not _load_profile().imu.enabled
        assert _load_sensor_config().seed == 17


@pytest.mark.parametrize("mode,pressure_rate", [("mathematical", 10.0), ("bag0402", 2.0)])
def test_named_modes_control_real_loaders_and_preserve_physics(mode, pressure_rate):
    original = {
        "ROS2_UUV_IMU_SENSOR_ENABLE": "0",
        "ROS2_UUV_BAR30_SENSOR_RATE_HZ": "99",
        "ROS2_UUV_DVL_SENSOR_CONFIG": "/nonexistent.json",
        "ROS2_UUV_DVL_SENSOR_SEED": "999",
        "SITL_REAL2SIM_BAG0402": "1",
        "ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE": "1",
    }
    env, snapshot = configure_sensor_error_mode(original, mode)
    assert original["ROS2_UUV_IMU_SENSOR_ENABLE"] == "0"
    assert env["SITL_REAL2SIM_BAG0402"] == "1"
    assert env["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] == "1"
    assert snapshot["replaced_overrides"]["ROS2_UUV_BAR30_SENSOR_RATE_HZ"] == "99"
    with patch.dict(os.environ, env, clear=True):
        imu = _load_profile()
        dvl = _load_sensor_config()
    assert imu.imu.enabled and imu.bar30.enabled and dvl.enabled
    assert imu.bar30.timing.schedule.rate_hz == pressure_rate
    assert dvl.seed == 2608
    assert dvl.dead_reckoning.attitude_source == "modeled_body_gyro"
    for item in snapshot["profiles"]:
        raw = Path(item["path"]).read_bytes()
        assert item["sha256"] == hashlib.sha256(raw).hexdigest()
        assert item["content"] == json.loads(raw)
        assert item["content"]["calibration_status"] == "unvalidated_prior"


@pytest.mark.parametrize("value", ["", "ideal", "../file", 123, {}])
def test_invalid_selection_is_rejected(value):
    with pytest.raises((ValueError, TypeError)):
        resolve_sensor_error_mode(value)


def test_named_mode_reset_to_existing_does_not_mutate_parent_environment():
    base = {"ROS2_UUV_IMU_SENSOR_SEED": "5"}
    configure_sensor_error_mode(base, "bag0402")
    env, _ = configure_sensor_error_mode(base, "existing")
    assert env == base
