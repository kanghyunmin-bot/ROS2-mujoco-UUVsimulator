"""Read real exported video/state/action through the organization U0 loader.

Set PYTHONPATH to the pinned auv_vla checkout and this collector package.
Inspection permits connection checks but never establishes demonstration quality.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from kmu26_auv_vla_data_collector.transfer_config import Kmu26TransferDataConfig
from gr00t.data.transform.base import ComposedModalityTransform
from kmu26_auv_vla_data_collector.training_input import Kmu26TrainingDataset

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("dataset", type=Path)
p.add_argument("--inspection", action="store_true")
p.add_argument("--output", type=Path)
a = p.parse_args()
config = Kmu26TransferDataConfig()
dataset = Kmu26TrainingDataset(
    dataset_path=a.dataset,
    modality_configs=config.modality_config(),
    embodiment_tag="new_embodiment",
    video_backend="decord",
    inspection=a.inspection,
)
raw = dataset[0]
assert raw["action.motion"].shape == (16, 4)
assert raw["video.ego"].dtype == np.uint8
assert raw["video.buoy_release"].dtype == np.uint8
# Exercise the actual organization's crop, resize, RGB and action normalization.
# The pinned repository bundles the processor; no policy weights are required.
transform = config.transform()
dataset.set_transforms_metadata(dataset.metadata)
pre_model = ComposedModalityTransform(transforms=transform.transforms[:-1])
pre_model.set_metadata(dataset.metadata)
prepared = pre_model(raw)
assert np.isfinite(np.asarray(prepared["action"])).all()
assert prepared["state"].shape == (1, 23)
transform.set_metadata(dataset.metadata)
model_input = transform(dataset[0])
assert model_input["state"].shape == (1, 64)
assert model_input["action"].shape == (16, 32)
assert int(model_input["state_mask"].sum()) == 23
assert int(model_input["action_mask"].sum()) == 16 * 4
# Read every usable sample to expose row/video indexing failures.
for i in range(len(dataset)):
    sample = dataset[i]
    assert sample["action.motion"].shape == (16, 4)
    assert all(
        np.isfinite(v).all() for k, v in sample.items() if k.startswith("state.")
    )
result = {
    "inspection_only": a.inspection,
    "usable_full_chunks": len(dataset),
    "first": {
        k: {"shape": list(v.shape), "dtype": str(v.dtype)} if hasattr(v, "shape") else v
        for k, v in dataset[0].items()
    },
    "pre_model": {k: list(v.shape) for k, v in prepared.items() if hasattr(v, "shape")},
    "model_input": {
        k: list(v.shape) for k, v in model_input.items() if hasattr(v, "shape")
    },
    "tail_padding": "excluded last 15 starts per episode",
    "optimizer_or_inference_run": False,
}
if a.output:
    a.output.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
