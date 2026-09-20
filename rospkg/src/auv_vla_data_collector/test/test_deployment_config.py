"""Deployment must retain CAP-off and the exact demonstrated RC contract."""
import json
from types import SimpleNamespace

import pytest

from kmu26_auv_vla_data_collector.deployment_config import (
    dataset_deployment_contract, disable_cap_for_checkpoint, validate_deployment_contract,
)


def data(tmp_path, span=400):
    directory = tmp_path / "meta"
    directory.mkdir(exist_ok=True)
    (directory / "source_manifests.jsonl").write_text(json.dumps({
        "fps": 10, "provenance": {"neutral_pwm": 1500, "pwm_span": span,
        "action_channels": [5, 6, 3, 4], "expected_mode": "STABILIZE"},
    }))
    return tmp_path


def test_old_span_is_not_silently_reinterpreted(tmp_path):
    with pytest.raises(ValueError, match="span=400"):
        dataset_deployment_contract(data(tmp_path, 300))


def test_cap_is_disabled_in_runtime_and_saved_config(tmp_path):
    model = SimpleNamespace(
        action_head=SimpleNamespace(config=SimpleNamespace(target_loss_weight=1.0)),
        config=SimpleNamespace(action_head_cfg={"target_loss_weight": 1.0}),
    )
    disable_cap_for_checkpoint(model)
    assert model.action_head.config.target_loss_weight == 0
    contract = dataset_deployment_contract(data(tmp_path))
    cfg = tmp_path / "experiment_cfg"
    cfg.mkdir()
    (cfg / "kmu26_transfer.json").write_text(json.dumps(contract))
    (tmp_path / "config.json").write_text(json.dumps({"action_head_cfg": model.config.action_head_cfg}))
    assert validate_deployment_contract(tmp_path) == contract
    model.config.action_head_cfg["target_loss_weight"] = 1
    (tmp_path / "config.json").write_text(json.dumps({"action_head_cfg": model.config.action_head_cfg}))
    with pytest.raises(ValueError, match="CAP-free"):
        validate_deployment_contract(tmp_path)


def test_mixed_controller_modes_rejected(tmp_path):
    data(tmp_path)
    p = tmp_path / "meta/source_manifests.jsonl"
    m = json.loads(p.read_text())
    m["provenance"]["expected_mode"] = "ALT_HOLD"
    p.write_text(p.read_text() + "\n" + json.dumps(m))
    with pytest.raises(ValueError, match="STABILIZE"):
        dataset_deployment_contract(tmp_path)
