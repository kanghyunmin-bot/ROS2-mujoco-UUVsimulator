"""Recording admission rejects mismatched commands and sparse pressure."""
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gui.collection_preflight import require_recordable_configuration, snapshot_files
from gui.sensor_error_mode import configure_sensor_error_mode


def test_bag_mode_rejected_before_recording():
    _, evidence = configure_sensor_error_mode({}, "bag0402")
    with pytest.raises(ValueError, match="수심"):
        require_recordable_configuration({"active_sensor_error": evidence}, 400)


def test_explicit_sparse_override_is_also_rejected():
    _, evidence = configure_sensor_error_mode({"ROS2_UUV_BAR30_SENSOR_RATE_HZ": "4"}, "existing")
    with pytest.raises(ValueError, match="수심"):
        require_recordable_configuration({"active_sensor_error": evidence}, 400)


def test_mismatched_span_rejected():
    with pytest.raises(ValueError, match="400"):
        require_recordable_configuration({}, 300)


def test_snapshot_is_immutable_when_source_changes(tmp_path):
    source = tmp_path / "profile.json"
    source.write_text('{"gain":1}')
    result = snapshot_files([source])
    source.write_text('{"gain":2}')
    assert json.loads(result[str(source)]["content"]) == {"gain": 1}
    assert result[str(source)]["sha256"] != snapshot_files([source])[str(source)]["sha256"]
