"""Persist the action and preprocessing contract with CAP-free checkpoints."""
from __future__ import annotations

import json
from pathlib import Path

TRANSFER_CONFIG = "kmu26_auv_vla_data_collector.transfer_config:Kmu26TransferDataConfig"


def dataset_deployment_contract(dataset: Path) -> dict:
    """Reject data whose command units or controller differ from sim deployment."""
    lines = (dataset / "meta/source_manifests.jsonl").read_text().splitlines()
    if not lines:
        raise ValueError("Empty training source manifests")
    for line in lines:
        manifest = json.loads(line)
        provenance = manifest["provenance"]
        expected = {"neutral_pwm": 1500, "pwm_span": 400,
                    "action_channels": [5, 6, 3, 4], "expected_mode": "STABILIZE"}
        if any(provenance.get(key) != value for key, value in expected.items()):
            raise ValueError("Training requires STABILIZE, channels [5,6,3,4], neutral=1500, span=400; remap/review old data explicitly")
        if manifest.get("fps") != 10:
            raise ValueError("Training and deployment require 10 Hz actions")
    return {"schema": "kmu26.cap_free.v1", **expected, "fps": 10,
            "data_config": TRANSFER_CONFIG, "target_loss_weight": 0.0}


def disable_cap_for_checkpoint(model) -> None:
    """Set both runtime and serialized action-head configs before checkpoint save."""
    model.action_head.config.target_loss_weight = 0.0
    model.config.action_head_cfg["target_loss_weight"] = 0.0


def validate_deployment_contract(checkpoint: Path) -> dict:
    """Reject incompatible preprocessing, command units or enabled CAP before GPU load."""
    config = json.loads((checkpoint / "experiment_cfg/kmu26_transfer.json").read_text())
    expected = {"schema": "kmu26.cap_free.v1", "neutral_pwm": 1500, "pwm_span": 400,
                "action_channels": [5, 6, 3, 4], "expected_mode": "STABILIZE", "fps": 10,
                "data_config": TRANSFER_CONFIG, "target_loss_weight": 0.0}
    if config != expected:
        raise ValueError("Checkpoint deployment contract differs from the reviewed simulator contract")
    model_config = json.loads((checkpoint / "config.json").read_text())
    if model_config.get("action_head_cfg", {}).get("target_loss_weight") != 0.0:
        raise ValueError("Checkpoint does not persist CAP-free inference")
    return config
