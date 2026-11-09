"""Exercise the full package workflow without the external DFKI download."""

import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from real2sim_auto import candidate_validation, read_package
from score_bluerov2_dfki import evaluate

TOOLS = Path(__file__).resolve().parent


def test_native_feedback_uses_per_trial_bias_and_guards_short_turns():
    from real2sim_native import AXIS_SCALES, refinement_profiles, select_refinement

    def score(biases, yaw_error, short_error, contact_count=0):
        return {"objective": 1., "contact_count": contact_count, "trials": {
            str(index): {
                "rmse": {**AXIS_SCALES, "Yaw": yaw_error},
                "yaw_rate_bias_deg_s": bias,
                "yaw_increment_rmse_deg": {"1": short_error},
            }
            for index, bias in enumerate(biases)
        }}

    symmetric = {"yaw_torque_scale": .4}
    directional = {"yaw_torque_thruster_scales": {"port": .6}, "yaw_damping_asymmetry": .4}
    base_score = score([2., -2.], 80, 10)
    proposed, diagnostics = refinement_profiles(
        {"symmetric": symmetric, "directional": directional},
        {"symmetric": base_score, "directional": score([-2., 2.], 80, 10)},
        "symmetric",
    )
    assert diagnostics["yaw_blend_from_native_bias"] == .5
    assert proposed["yaw_bias_step"]["yaw_torque_thruster_scales"]["port"] == pytest.approx(.5)
    selected, assessed = select_refinement({
        "symmetric": base_score,
        "bad_short_turn": score([.1, -.1], 10, 11),
        "contact": score([.1, -.1], 10, 10, contact_count=1),
        "balanced": score([1., -1.], 50, 10.2),
    }, "symmetric")
    assert selected == "balanced"
    assert assessed["bad_short_turn"]["rejected_by"] == ["yaw_1s"]
    assert assessed["contact"]["rejected_by"] == ["contacts"]


def test_coarse_yaw_asymmetry_and_motor_moments_match_runtime_conventions():
    from real2sim_fit import _motor_wrench, _simulate

    parameters = np.zeros(20)
    parameters[5] = 1
    for rate in (-.4, .4):
        initial = np.zeros(10)
        initial[5] = rate
        args = (initial, np.zeros((2, 6)), parameters, 10., np.ones(3), np.zeros(3))
        symmetric = abs(_simulate(*args)[1, 5])
        directional = abs(_simulate(*args, .5)[1, 5])
        assert (directional - symmetric) * rate < 0

    geometry = np.zeros((2, 6))
    geometry[:, 5] = [1, 2]
    thrusters = [{"name": "drive", "kind": "horizontal"}, {"name": "lift", "kind": "vertical"}]
    wrench = _motor_wrench(np.ones((1, 2)), geometry, np.ones(2), thrusters, .8, {"drive": .3})
    assert wrench[0, 5] == pytest.approx(2.3)


def test_build_with_custom_motor_and_independent_holdout(tmp_path):
    package = tmp_path / "input"
    model = package / "model"
    model.mkdir(parents=True)
    (model / "scene.xml").write_text(
        """<mujoco>
        <option timestep=".002"/>
        <worldbody><body name="base_link" pos="0 0 -2">
          <freejoint name="world_joint"/>
          <inertial pos="0 0 0" mass="10" diaginertia=".1 .2 .25"/>
          <geom type="sphere" size=".1"/>
          <site name="thr_drive_a" pos="0 0 0"/>
        </body></worldbody>
        <actuator><motor name="drive_a" site="thr_drive_a" gear="1 0 0 0 0 0"
          ctrllimited="true" ctrlrange="-100 100"/></actuator>
        </mujoco>"""
    )
    perf = TOOLS.parent / "config" / "thruster_performance.json"
    (package / "thruster.json").write_bytes(perf.read_bytes())
    (package / "profiles.json").write_text(json.dumps({
        "synthetic_nominal": {
            "buoyancy_model": "ellipsoid", "half_height": .1, "cob_z_offset": .02,
            "thruster_response_model": "rpm", "linear_damping_linear": [1, 2, 3],
        },
    }))
    (package / "vehicle.json").write_text(json.dumps({
        "version": 1, "name": "test_rov", "scene": "model/scene.xml",
        "profile_file": "profiles.json", "base_profile": "synthetic_nominal",
        "thruster_perf_file": "thruster.json",
        "thrusters": [{"name": "drive_a", "kind": "horizontal", "pwm_sign": 1}],
        "sensors": {"dvl": {"pos_m": [0, 0, -.1], "quat_wxyz": [0, 1, 0, 0]}},
        "train": ["train"], "holdout": ["holdout"],
    }))
    for role, speed in (("train", .11), ("holdout", .14)):
        trial = package / "trials" / role
        trial.mkdir(parents=True)
        (trial / "metadata.json").write_text('{"initial_depth_m": 2.0, "initial_depth_reference": "base_link"}')
        t = np.arange(0.0, 4.1, .1)
        zeros = np.zeros(len(t))
        real = np.column_stack([t, speed * np.sin(t), *([zeros] * 5), zeros + 2, *([zeros] * 3)])
        for name, values, header in (
            ("real", real, "t_s,surge_mps,sway_mps,heave_mps,roll_rate_radps,pitch_rate_radps,"
                           "yaw_rate_radps,depth_m,roll_deg,pitch_deg,yaw_deg"),
            ("pwm", np.column_stack((t, 1500 + 100 * (t > 1))), "t_s,pwm1"),
            ("voltage", np.column_stack((t, zeros + 16)), "sim_time,voltage_v"),
        ):
            np.savetxt(trial / f"{name}.csv", values, header=header, delimiter=",", comments="", fmt="%.4f")

    output = tmp_path / "output"
    result = subprocess.run([
        sys.executable, str(TOOLS / "real2sim_auto.py"), "run", str(package),
        "--output", str(output), "--headless", "--max-evaluations", "2",
    ], capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads((output / "report.json").read_text())
    assert report["fit"]["train_trials"] == ["train"]
    selection = report["fit"]["native_selection"]
    assert selection["training_only"]
    assert selection["selected"] in selection["candidates"]
    assert all(set(entry["trials"]) == {"train"} for entry in selection["candidates"].values())
    assert len(report["comparison"]["holdout"]["metrics"]) == 6
    assert report["trials"]["holdout"]["path"] == "../input/trials/holdout"
    assert "dvl" in json.loads((output / "sensor_config.json").read_text())

    replays = output / "replays" / "holdout"
    candidate_run = replays / "candidate" / "run.json"
    metadata = json.loads(candidate_run.read_text())
    candidate_run.write_text(json.dumps({**metadata, "first_contact_s": 2.0}))
    scores, traces = evaluate(
        package / "trials/holdout/real.csv", [replays / "nominal", replays / "candidate"],
        measured_max_gap_s=.25,
    )
    assert scores["free_motion"]["common_end_exclusive_s"] == 2.0
    pre_contact = traces["surge_valid"] & (traces["t_s"] < 2.0)
    assert scores["free_motion"]["metrics"][0]["samples"] == int(pre_contact.sum())
    assert 10 <= pre_contact.sum() < scores["metrics"][0]["samples"]
    assert candidate_validation({"holdout": scores}) == {
        "candidate_requires_review": True, "candidate_first_contact_s": {"holdout": 2.0},
    }

    (package / "trials/holdout/real.csv").write_bytes((package / "trials/train/real.csv").read_bytes())
    with pytest.raises(ValueError, match="Measured CSV is identical"):
        read_package(package)
