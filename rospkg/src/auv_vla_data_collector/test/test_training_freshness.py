"""Training admission uses the recorded clock and independent motion capture."""

import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType

import numpy as np
import pandas as pd
import pytest


@pytest.fixture
def validator(monkeypatch):
    # Only the optional U0 base class is stubbed; validation reads real parquet
    # and acquisition files. The actual U0 loader is checked separately.
    for name in ("gr00t", "gr00t.data", "gr00t.data.dataset"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules["gr00t.data.dataset"].LeRobotSingleDataset = object
    path = Path(__file__).parents[1] / "kmu26_auv_vla_data_collector/training_input.py"
    spec = importlib.util.spec_from_file_location("training_freshness_subject", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.validate_demonstrations


def dataset(root, source="simulation", wall=4.0, ros=0.5, raw_age=None):
    provenance = dict(collection_kind="task_demonstration", data_source=source,
                      session_id="test", context={"test": True}, expected_mode="MANUAL",
                      use_sim_time=source == "simulation", max_sensor_age_sec=0.25)
    rows = [dict(connected=True, armed=True, mode="MANUAL", rc_publishers=1,
                 state_receipt_age_wall_s=wall, state_receipt_age_ros_s=ros) for _ in range(2)]
    if raw_age is not None:
        provenance["imu_motion"] = dict(topic="/mavros/imu/data_raw", frame="fcu_link", convention="FLU")
        for i, row in enumerate(rows):
            row["imu_motion"] = dict(source_time=10 + i / 10 - raw_age,
                                     receipt_time=10 + i / 10, frame_id="fcu_link")
    manifest = dict(frames=2, provenance=provenance, termination_reason="operator_stop", success=True)
    meta = root / "meta"
    audit = meta / "acquisition/episode_000000"
    audit.mkdir(parents=True)
    (meta / "source_manifests.jsonl").write_text(json.dumps(manifest) + "\n")
    (audit / "vehicle_state.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    data = root / "data/chunk-000"
    data.mkdir(parents=True)
    pd.DataFrame({"observation.state": [np.ones(23)] * 2,
                  "telemetry.source_timestamp": [np.full(7, 10), np.full(7, 10.1)],
                  "telemetry.ros_timestamp": [10.0, 10.1]}).to_parquet(data / "episode_000000.parquet")


def test_simulation_uses_ros_age_despite_slow_wall_clock(tmp_path, validator):
    dataset(tmp_path)
    validator(tmp_path)


@pytest.mark.parametrize("age", [None, float("nan"), -0.1, 2.1])
def test_simulation_rejects_unverified_ros_age(tmp_path, validator, age):
    dataset(tmp_path, wall=0.1, ros=age)
    with pytest.raises(ValueError, match="Vehicle/control"):
        validator(tmp_path)


def test_physical_keeps_wall_clock_boundary(tmp_path, validator):
    dataset(tmp_path, source="real", wall=2.1, ros=0.1)
    with pytest.raises(ValueError, match="Vehicle/control"):
        validator(tmp_path)


def test_physical_does_not_need_simulation_age(tmp_path, validator):
    dataset(tmp_path, source="real", wall=0.1, ros=None)
    validator(tmp_path)


@pytest.mark.parametrize("age", [0.1, 0.3])
def test_raw_motion_has_its_own_freshness_boundary(tmp_path, validator, age):
    dataset(tmp_path, wall=0.1, raw_age=age)
    if age > 0.25:
        with pytest.raises(ValueError, match="IMU motion"):
            validator(tmp_path)
    else:
        validator(tmp_path)
