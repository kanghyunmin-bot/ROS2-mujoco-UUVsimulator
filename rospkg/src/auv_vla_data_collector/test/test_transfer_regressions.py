"""Regression fixtures for transfer data loss and unsafe episode continuity."""

import json

import cv2
import numpy as np
import pytest

from kmu26_auv_vla_data_collector.export_lerobot import export_dataset


def episode(tmp_path):
    root = tmp_path / "staging"
    ep = root / "episode_000000"
    for camera in ("ego", "buoy_release"):
        frames = ep / "frames" / camera
        frames.mkdir(parents=True)
        for i in range(3):
            cv2.imwrite(
                str(frames / f"frame_{i:06d}.jpg"), np.zeros((8, 8, 3), np.uint8)
            )
    state = np.zeros((3, 23), np.float32)
    state[:, 13] = 1
    state[:, 19:] = 1
    values = dict(
        observation_state=state,
        action=np.zeros((3, 4), np.float32),
        rc_pwm=np.full((3, 4), 1500, np.int32),
        rc_update_mask=np.ones((3, 4), np.float32),
        ros_timestamp=np.arange(3) / 10,
        source_age=np.zeros((3, 7), np.float32),
        source_timestamp=np.zeros((3, 7), np.float64),
    )
    np.savez(ep / "samples.npz", **values)
    manifest = dict(
        episode_index=0,
        frames=3,
        fps=10,
        task="Check connection.",
        success=False,
        termination_reason="node_shutdown",
        provenance={"data_source": "simulation"},
    )
    (ep / "manifest.json").write_text(json.dumps(manifest))
    return root, ep, values


def test_nonfinite_state_rejected_before_output(tmp_path):
    root, ep, values = episode(tmp_path)
    values["observation_state"][1, 5] = np.nan
    np.savez(ep / "samples.npz", **values)
    with pytest.raises(ValueError, match="state"):
        export_dataset(root, tmp_path / "out", None)
    assert not (tmp_path / "out").exists()


def test_noncontiguous_image_names_rejected_before_output(tmp_path):
    root, ep, _ = episode(tmp_path)
    frames = ep / "frames" / "ego"
    (frames / "frame_000001.jpg").rename(frames / "frame_000003.jpg")
    with pytest.raises(ValueError, match="contiguous"):
        export_dataset(root, tmp_path / "out", None)
    assert not (tmp_path / "out").exists()


def test_export_preserves_source_and_termination(tmp_path):
    root, _, _ = episode(tmp_path)
    export_dataset(root, tmp_path / "out", None)
    exported = json.loads((tmp_path / "out/meta/episodes.jsonl").read_text())
    assert exported["termination_reason"] == "node_shutdown"
    assert exported["provenance"]["data_source"] == "simulation"
    info = json.loads((tmp_path / "out/meta/info.json").read_text())
    assert info["splits"]["train"] == "0:1"


def test_declared_state_order_must_match_loader(tmp_path):
    from kmu26_auv_vla_data_collector.contract import STATE_NAMES

    root, ep, _ = episode(tmp_path)
    path = ep / "manifest.json"
    m = json.loads(path.read_text())
    m["state_names"] = list(reversed(STATE_NAMES))
    path.write_text(json.dumps(m))
    with pytest.raises(ValueError, match="state_names"):
        export_dataset(root, tmp_path / "out", None)


@pytest.mark.parametrize("key", ["source_timestamp", "receipt_timestamp"])
def test_nonfinite_sensor_timestamp_rejected_before_output(tmp_path, key):
    root, ep, values = episode(tmp_path)
    values[key] = np.zeros((3, 7), np.float64)
    values[key][1, 0] = np.nan
    np.savez(ep / "samples.npz", **values)
    with pytest.raises(ValueError, match="timestamp"):
        export_dataset(root, tmp_path / "out", None)
    assert not (tmp_path / "out").exists()
