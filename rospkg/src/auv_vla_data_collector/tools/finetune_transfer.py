"""Invoke the pinned U0 trainer with reviewed data and complete future chunks.

Example: python finetune_transfer.py /path/to/auv_vla --dataset-path /path/to/train ...
This entry point deliberately accepts one preselected dataset root. Mixed-source
weighting and validation session selection must be decided before this step.
"""

import importlib.util
import sys
from pathlib import Path

from kmu26_auv_vla_data_collector.training_input import Kmu26TrainingDataset

root = Path(sys.argv.pop(1)).resolve()
spec = importlib.util.spec_from_file_location(
    "kmu26_trainer", root / "scripts/gr00t_finetune.py"
)
trainer = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = trainer
spec.loader.exec_module(trainer)
trainer.LeRobotSingleDataset = Kmu26TrainingDataset
import tyro

config = tyro.cli(trainer.ArgsConfig)
if len(config.dataset_path) != 1:
    raise ValueError(
        "Choose mixture weights/statistics explicitly; this entry point takes one root"
    )
config.data_config = (
    "kmu26_auv_vla_data_collector.transfer_config:Kmu26TransferDataConfig"
)
config.video_backend = "decord"
trainer.main(config)
