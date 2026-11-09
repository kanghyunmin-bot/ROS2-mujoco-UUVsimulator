"""Tests for discovery and safe viewer launch from the web GUI."""

import json
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from gui.web_real2sim import WebReal2Sim


@pytest.fixture
def output(tmp_path: Path):
    folder = tmp_path / "blue"
    folder.mkdir()
    report = {"vehicle": "BlueROV2", "scene": "model/scene.xml", "trials": {"trial": {}},
              "variants": {"nominal": "initial", "candidate": "fitted"}}
    (folder / "report.json").write_text(json.dumps(report))
    (folder / "sensor_config.json").write_text(json.dumps({"imu": {}, "pressure": {}}))
    for name in ("vehicle.json", "profiles.json", "thruster_performance.json"):
        (folder / name).write_text("{}")
    (folder / "model").mkdir()
    (folder / "model/scene.xml").write_text("<mujoco/>")
    return folder


def test_discover_and_start_only_selected_vehicle(output):
    manager = WebReal2Sim(output.parent)
    vehicle = manager.vehicles()[0]
    assert vehicle["name"] == "BlueROV2"
    assert vehicle["sensors"] == ["imu", "pressure"]
    assert vehicle["trials"] == ["trial"]
    process = MagicMock(pid=123)
    process.poll.return_value = None
    with patch("gui.web_real2sim.subprocess.Popen", return_value=process) as launch:
        assert manager.start({"id": vehicle["id"], "trial": "trial", "profile": "candidate"})["running"]
        command = launch.call_args.args[0]
        assert command[2:4] == ["view", str(output)]
        assert "--sitl" not in command
        with pytest.raises(ValueError, match="already running"):
            manager.start({"id": vehicle["id"]})
    with pytest.raises(ValueError, match="Unknown"):
        manager.start({"id": "../../etc/passwd"})


def test_incomplete_and_escaping_scenes_are_not_discovered(output):
    manager = WebReal2Sim(output.parent)
    assert len(manager.vehicles()) == 1
    report = json.loads((output / "report.json").read_text())
    report["scene"] = "../bad.xml"
    (output / "report.json").write_text(json.dumps(report))
    assert manager.vehicles() == []


def test_build_rejects_unsafe_paths_and_runs_bounded_job(tmp_path):
    package = tmp_path / "input"
    package.mkdir()
    (package / "vehicle.json").write_text("{}")
    manager = WebReal2Sim(root=tmp_path / "outputs", workspace=tmp_path)
    for payload in ({"package": "../outside", "output_name": "ok"},
                    {"package": str(package), "output_name": "ok"},
                    {"package": "input", "output_name": "../outside"}):
        with pytest.raises(ValueError):
            manager.build(payload)
    process = MagicMock(pid=456)
    process.poll.return_value = None
    with patch("gui.web_real2sim.subprocess.Popen", return_value=process) as launch:
        result = manager.build({"package": "input", "output_name": "result"})
        assert result["running"] and result["output"] == "result"
        command = launch.call_args.args[0]
        assert command[2:4] == ["run", str(package)]
        assert "--headless" in command and command[-4:] == ["--jobs", "2", "--max-evaluations", "35"]
        assert manager.build_status()["success"] is None
        with pytest.raises(ValueError, match="already running"):
            manager.build({"package": "input", "output_name": "second"})
    process.poll.return_value = 0
    assert manager.build_status()["success"] is False  # no completed report
    (tmp_path / "outputs/result").mkdir()
    (tmp_path / "outputs/result/report.json").write_text("{}")
    assert manager.build_status()["success"] is True
    with pytest.raises(ValueError, match="already exists"):
        manager.build({"package": "input", "output_name": "result"})


def test_discovery_exposes_candidate_review(output):
    report_file = output / "report.json"
    report = json.loads(report_file.read_text())
    report["validation"] = {"candidate_requires_review": True}
    report_file.write_text(json.dumps(report))
    assert WebReal2Sim(output.parent).vehicles()[0]["needs_review"] is True


def test_build_subprocess_runs_and_reports_completion(tmp_path, monkeypatch):
    import time

    package = tmp_path / "input"
    package.mkdir()
    (package / "vehicle.json").write_text("{}")
    fake_python = tmp_path / "fake-python"
    fake_python.write_text("#!/usr/bin/env python3\nimport pathlib, sys\np = pathlib.Path(sys.argv[sys.argv.index('--output') + 1])\np.mkdir()\n(p / 'report.json').write_text('{}')\nprint('finished')\n")
    fake_python.chmod(0o755)
    monkeypatch.setenv("UUV_REAL2SIM_PYTHON", str(fake_python))
    manager = WebReal2Sim(root=tmp_path / "outputs", workspace=tmp_path)
    manager.build({"package": "input", "output_name": "result"})
    deadline = time.monotonic() + 5
    while manager.build_status()["running"] and time.monotonic() < deadline:
        time.sleep(.01)
    status = manager.build_status()
    assert status["success"] is True and "finished" in status["log"]
