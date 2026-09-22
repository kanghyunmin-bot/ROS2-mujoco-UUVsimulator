"""Invoke the pinned U0 trainer with reviewed data and complete future chunks.

Example: python finetune_transfer.py /path/to/auv_vla --dataset-path /path/to/train ...
This entry point deliberately accepts one preselected dataset root. Mixed-source
weighting and validation session selection must be decided before this step.
"""

import importlib.util
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
root = Path(sys.argv.pop(1)).resolve()
annotations = None
if "--interaction-annotations" in sys.argv:
    index = sys.argv.index("--interaction-annotations")
    annotations = Path(sys.argv.pop(index + 1)).resolve()
    sys.argv.pop(index)
# Explicit opt-in for 16 GB GPUs; retain upstream defaults for other callers.
model_storage_bf16 = "--model-storage-bf16" in sys.argv
if model_storage_bf16:
    sys.argv.remove("--model-storage-bf16")
# Use the supplied organization fork, not an unrelated installed GR00T package.
sys.path.insert(0, str(root))
from kmu26_auv_vla_data_collector.training_input import Kmu26TrainingDataset
from kmu26_auv_vla_data_collector.deployment_config import (
    dataset_deployment_contract, disable_cap_for_checkpoint,
)
spec = importlib.util.spec_from_file_location(
    "kmu26_trainer", root / "scripts/gr00t_finetune.py"
)
trainer = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = trainer
spec.loader.exec_module(trainer)
trainer.LeRobotSingleDataset = Kmu26TrainingDataset
if model_storage_bf16:
    import torch
    original_from_pretrained = trainer.GR00T_N1_5.from_pretrained

    def load_bf16(*args, **kwargs):
        kwargs["torch_dtype"] = torch.bfloat16
        model = original_from_pretrained(*args, **kwargs)
        # HF's bf16 construction context also changes the unregistered Beta
        # distribution. PyTorch samples Dirichlet only in float32/float64;
        # upstream sample_time already casts the sampled times afterwards.
        beta = model.action_head.beta_dist
        model.action_head.beta_dist = torch.distributions.Beta(
            beta.concentration1.float(), beta.concentration0.float()
        )
        return model

    trainer.GR00T_N1_5.from_pretrained = staticmethod(load_bf16)
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
# This entry point owns the CAP-free experiment; target labels are unavailable.
config.target_loss_weight = 0.0
print("KMU26 transfer: CAP target branch disabled (target_loss_weight=0)")
contract = dataset_deployment_contract(Path(config.dataset_path[0]))
evidence_dir = Path(config.output_dir) / "experiment_cfg"
evidence_dir.mkdir(parents=True, exist_ok=True)
if annotations is not None:
    import numpy as np
    from gr00t.experiment.trainer import DualBrainTrainer
    from kmu26_auv_vla_data_collector.weighted_sampling import (
        ReviewedWindowSampler, chunk_weights,
    )

    def reviewed_train_sampler(self):
        weights = chunk_weights(self.train_dataset, annotations)
        summary = {
            "chunks": len(weights), "boosted_chunks": int(np.sum(weights > 1)),
            "min_weight": float(weights.min()), "max_weight": float(weights.max()),
            "boosted_probability": float(weights[weights > 1].sum() / weights.sum()),
            "uniform_boosted_probability": float(np.mean(weights > 1)),
            "seed": self.args.seed, "replacement": True,
            "scope": "visual proximity only; no contact/detach ground truth",
        }
        (evidence_dir / "sampling_summary.json").write_text(json.dumps(summary, indent=2))
        print("Reviewed interaction sampling:", summary, flush=True)
        return ReviewedWindowSampler(self.train_dataset, weights, self.args.seed)

    DualBrainTrainer._get_train_sampler = reviewed_train_sampler
    (evidence_dir / "interaction_annotations.json").write_bytes(annotations.read_bytes())
(evidence_dir / "kmu26_transfer.json").write_text(json.dumps(contract, indent=2) + "\n")
original_runner = trainer.TrainRunner


def cap_free_runner(*args, **kwargs):
    model = kwargs["model"] if "model" in kwargs else args[0]
    disable_cap_for_checkpoint(model)
    return original_runner(*args, **kwargs)


trainer.TrainRunner = cap_free_runner
trainer.main(config)
