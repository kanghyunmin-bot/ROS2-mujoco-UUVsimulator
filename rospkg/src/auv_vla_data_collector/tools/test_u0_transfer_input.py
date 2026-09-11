"""Run in the optional U0 environment; no model weights are allocated."""

import os
from pathlib import Path
import numpy as np
import pytest

pytest.importorskip("gr00t")
from gr00t.experiment.data_config import Kmu26AuvRealDataConfig
from gr00t.data.transform.base import ComposedModalityTransform
from kmu26_auv_vla_data_collector.transfer_config import Kmu26TransferDataConfig
from kmu26_auv_vla_data_collector.training_input import Kmu26TrainingDataset


@pytest.fixture
def dataset():
    root = os.environ.get("KMU26_TEST_DATASET")
    if not root:
        pytest.skip("Set KMU26_TEST_DATASET to an exported connection check")
    config = Kmu26TransferDataConfig()
    return Kmu26TrainingDataset(
        dataset_path=Path(root),
        modality_configs=config.modality_config(),
        embodiment_tag="new_embodiment",
        video_backend="decord",
        inspection=True,
    )


def test_linear_states_are_not_angular_features(dataset):
    config = (
        Kmu26AuvRealDataConfig()
        if os.environ.get("KMU26_TEST_WITHOUT_FIX")
        else Kmu26TransferDataConfig()
    )
    transform = ComposedModalityTransform(transforms=config.transform().transforms[:-1])
    transform.set_metadata(dataset.metadata)
    obs = dataset[0]
    expected = np.concatenate([obs[k] for k in config.state_keys], axis=-1)
    result = transform(obs)
    np.testing.assert_allclose(result["state"], expected)


def test_last_start_contains_exact_future_actions(dataset):
    # 50 frames -> 35 complete starts; terminal repeat padding never reaches loss.
    assert len(dataset) == sum(max(0, int(n) - 15) for n in dataset.trajectory_lengths)
    tid, start = dataset.all_steps[-1]
    assert start + 15 == int(dataset.trajectory_lengths[-1]) - 1
    assert dataset[-1]["action.motion"].shape == (16, 4)


def test_connection_check_rejected_for_training(dataset):
    with pytest.raises(ValueError, match="demonstration"):
        Kmu26TrainingDataset(
            dataset_path=dataset.dataset_path,
            modality_configs=dataset.modality_configs,
            embodiment_tag="new_embodiment",
            video_backend="decord",
        )


def test_same_session_cannot_leak_into_validation(tmp_path):
    import json
    from kmu26_auv_vla_data_collector.training_input import assert_disjoint_sessions

    for name in ("train", "validation"):
        meta = tmp_path / name / "meta"
        meta.mkdir(parents=True)
        (meta / "source_manifests.jsonl").write_text(
            json.dumps({"provenance": {"session_id": "same-session"}}) + "\n"
        )
    with pytest.raises(ValueError, match="overlap"):
        assert_disjoint_sessions(tmp_path / "train", tmp_path / "validation")


def test_training_rejects_nonfinite_camera_timestamp(tmp_path):
    import json
    import pandas as pd
    from kmu26_auv_vla_data_collector.training_input import validate_demonstrations

    meta = tmp_path / "meta"
    acquisition = meta / "acquisition" / "episode_000000"
    acquisition.mkdir(parents=True)
    manifest = {
        "frames": 3,
        "success": True,
        "termination_reason": "operator_stop",
        "provenance": {
            "collection_kind": "task_demonstration",
            "data_source": "simulation",
            "session_id": "synthetic-regression",
            "context": {"fixture": True},
            "expected_mode": "STABILIZE",
        },
    }
    (meta / "source_manifests.jsonl").write_text(json.dumps(manifest) + "\n")
    vehicle = {
        "connected": True,
        "armed": True,
        "mode": "STABILIZE",
        "rc_publishers": 1,
        "state_receipt_age_wall_s": 0.1,
    }
    (acquisition / "vehicle_state.jsonl").write_text((json.dumps(vehicle) + "\n") * 3)
    data = tmp_path / "data" / "chunk-000"
    data.mkdir(parents=True)
    state = np.zeros((3, 23))
    state[:, 19:21] = 1
    stamps = np.tile(np.arange(3)[:, None] / 10, (1, 7))
    stamps[1, 0] = np.nan
    pd.DataFrame(
        {"observation.state": list(state), "telemetry.source_timestamp": list(stamps)}
    ).to_parquet(data / "episode_000000.parquet")
    with pytest.raises(ValueError, match="invalid"):
        validate_demonstrations(tmp_path)
