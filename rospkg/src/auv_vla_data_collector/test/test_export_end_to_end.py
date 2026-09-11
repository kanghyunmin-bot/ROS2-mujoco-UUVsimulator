import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from kmu26_auv_vla_data_collector.export_lerobot import export_dataset

pd = pytest.importorskip("pandas")
pytest.importorskip("pyarrow")


def _write_frames(directory: Path, count: int, shape: tuple[int, int, int]) -> None:
    directory.mkdir(parents=True)
    for index in range(count):
        image = np.full(shape, index * 20, dtype=np.uint8)
        assert cv2.imwrite(str(directory / f"frame_{index:06d}.jpg"), image)


def test_export_dataset_creates_u0_compatible_layout(tmp_path):
    staging = tmp_path / "staging"
    episode = staging / "episode_000007"
    _write_frames(episode / "frames" / "ego", 3, (8, 12, 3))
    _write_frames(episode / "frames" / "buoy_release", 3, (6, 10, 3))
    np.savez_compressed(
        episode / "samples.npz",
        observation_state=np.zeros((3, 23), dtype=np.float32),
        action=np.zeros((3, 4), dtype=np.float32),
        rc_pwm=np.full((3, 4), 1500, dtype=np.int32),
        rc_update_mask=np.ones((3, 4), dtype=np.float32),
        ros_timestamp=np.arange(3, dtype=np.float64) / 10.0,
        source_age=np.zeros((3, 7), dtype=np.float32),
        source_timestamp=np.zeros((3, 7), dtype=np.float64),
    )
    (episode / "manifest.json").write_text(
        json.dumps(
            {
                "episode_index": 7,
                "task": "Approach the red buoy.",
                "success": True,
                "frames": 3,
                "fps": 10.0,
            }
        )
    )

    output = tmp_path / "lerobot"
    export_dataset(staging, output, requested_fps=None)

    table = pd.read_parquet(output / "data" / "chunk-000" / "episode_000000.parquet")
    assert len(table) == 3
    assert np.stack(table["observation.state"]).shape == (3, 23)
    assert np.stack(table["action"]).shape == (3, 4)
    info = json.loads((output / "meta" / "info.json").read_text())
    assert info["features"]["observation.images.ego"]["shape"] == [8, 12, 3]
    assert info["features"]["observation.images.buoy_release"]["shape"] == [6, 10, 3]
