"""Regression checks for recording defaults, action units and CAP-free training."""
import json
from pathlib import Path
import runpy
import sys
from types import ModuleType, SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
CURRENT = ROOT / "uuv_mujoco/current"
sys.path.insert(0, str(CURRENT))


def test_default_camera_is_recordable():
    from gui.sim_stack_launch_command import camera_config_from_owner, normalize_camera_config
    assert normalize_camera_config()["hz"] >= 10
    assert camera_config_from_owner(SimpleNamespace(env={}))["hz"] >= 10


def test_sim_policy_reconstructs_full_gui_action_range():
    from gui.config_rc import RC_PWM_SPAN
    cfg = yaml.safe_load((ROOT / "rospkg/src/kmu26_auv_vla_policy/config/sim_policy.yaml").read_text())
    assert cfg["vla_data_collector"]["ros__parameters"]["pwm_span"] == RC_PWM_SPAN
    assert cfg["vla_policy"]["ros__parameters"]["command_limit"] == 1.0
    assert cfg["vla_policy"]["ros__parameters"]["dry_run"] is True


def test_finetune_entry_disables_cap_before_trainer(tmp_path, monkeypatch):
    # Exercise the real entry point without a GPU: the trainer records received settings.
    data = tmp_path / "data/meta"
    data.mkdir(parents=True)
    (data / "source_manifests.jsonl").write_text(json.dumps({
        "fps": 10, "provenance": {"neutral_pwm":1500, "pwm_span":400,
        "action_channels":[5,6,3,4], "expected_mode":"STABILIZE"}}) + "\n")
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    result = tmp_path / "args.json"
    (scripts / "gr00t_finetune.py").write_text(
        "import json\n"
        "class ArgsConfig: pass\n"
        "TrainRunner = lambda *a, **k: None\n"
        f"def main(config):\n    open({str(result)!r}, 'w').write(json.dumps(vars(config)))\n"
    )
    package = ModuleType("kmu26_auv_vla_data_collector")
    package.__path__ = [str(ROOT / "rospkg/src/auv_vla_data_collector/kmu26_auv_vla_data_collector")]
    module = ModuleType("kmu26_auv_vla_data_collector.training_input")
    module.Kmu26TrainingDataset = object
    monkeypatch.setitem(sys.modules, package.__name__, package)
    monkeypatch.setitem(sys.modules, module.__name__, module)
    tyro = ModuleType("tyro")
    tyro.cli = lambda *a, **k: SimpleNamespace(dataset_path=[str(data.parent)], output_dir=str(tmp_path / "model"), target_loss_weight=1.0)
    monkeypatch.setitem(sys.modules, "tyro", tyro)
    monkeypatch.setattr(sys, "argv", ["finetune_transfer.py", str(tmp_path)])
    runpy.run_path(str(ROOT / "rospkg/src/auv_vla_data_collector/tools/finetune_transfer.py"), run_name="__main__")
    assert json.loads(result.read_text())["target_loss_weight"] == 0.0


def test_research_pool_has_one_yellow_buoy():
    import xml.etree.ElementTree as ET
    root = ET.parse(CURRENT / "scenes/research_pool_slam_scene.xml")
    names = [b.get("name") for b in root.findall(".//body") if b.get("name", "").startswith("course_buoy_") and b.get("name", "").endswith("_float")]
    assert names == ["course_buoy_a_yellow_1_float"]
