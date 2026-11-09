"""Regression checks for target-trial native yaw refinement."""

import json
from pathlib import Path

import numpy as np
import pytest

import real2sim_target_yaw as target_yaw


def test_target_fit_converges_without_mutating_input_or_using_holdout(tmp_path: Path, monkeypatch) -> None:
    names = ["a", "b", "c", "d"]
    (tmp_path / "vehicle.json").write_text(json.dumps({
        "thrusters": [{"name": name, "kind": "horizontal"} for name in names]
    }))
    state = {"trials": {
        "training": {"split": "train", "path": "."},
        "holdout": {"split": "holdout", "path": "."},
    }}
    original = {"yaw_torque_scale": .5, "linear_damping_angular": [1., 2., 3.]}
    target = np.array([.3, .6, .4, .7, 3.5, .7, .3])
    saved = {}

    def fake_evaluate(output, state, profiles, trial_names, *, jobs, runs_dir):
        assert trial_names == ["training"]
        result = {}
        for label, profile in profiles.items():
            values = np.array([profile["yaw_torque_thruster_scales"][name] for name in names]
                              + [profile["linear_damping_angular"][2], profile["quadratic_damping_angular"][2],
                                 profile["yaw_damping_asymmetry"]])
            saved[label] = values
            result[label] = {"contact_count": 0, "trials": {"training": {
                "rmse": {"Yaw": float(np.linalg.norm(values - target))}, "first_contact_s": None}}}
        return result

    monkeypatch.setattr(target_yaw, "evaluate_candidates", fake_evaluate)
    monkeypatch.setattr(target_yaw, "_residual", lambda output, state, trial, run: saved[run.parent.name] - target)
    label, fitted, diagnostics = target_yaw.fit_target_yaw(
        tmp_path, state, original, "training", jobs=2, runs_dir=tmp_path / "runs", rounds=3,
    )
    actual = np.array([fitted["yaw_torque_thruster_scales"][name] for name in names]
                      + [fitted["linear_damping_angular"][2], fitted["quadratic_damping_angular"][2],
                         fitted["yaw_damping_asymmetry"]])
    np.testing.assert_allclose(actual, target, atol=.01)
    assert original == {"yaw_torque_scale": .5, "linear_damping_angular": [1., 2., 3.]}
    assert diagnostics["training_trial"] == "training"
    assert label == diagnostics["selected"]
    with pytest.raises(ValueError, match="declared training"):
        target_yaw.fit_target_yaw(tmp_path, state, original, "holdout", jobs=2, runs_dir=tmp_path / "runs", rounds=1)


def test_target_fit_rejects_contact_even_when_yaw_error_improves(tmp_path: Path, monkeypatch) -> None:
    (tmp_path / "vehicle.json").write_text(json.dumps({"thrusters": [{"name": "a", "kind": "horizontal"}]}))
    state = {"trials": {"training": {"split": "train", "path": "."}}}
    profiles = {}

    def fake_evaluate(output, state, candidates, trial_names, *, jobs, runs_dir):
        result = {}
        for label, profile in candidates.items():
            values = np.array([profile["yaw_torque_thruster_scales"]["a"],
                               profile["linear_damping_angular"][2], profile["quadratic_damping_angular"][2],
                               profile["yaw_damping_asymmetry"]])
            profiles[label] = values
            contact = 1.0 if "_step" in label else None
            result[label] = {"contact_count": int(contact is not None), "trials": {"training": {
                "rmse": {"Yaw": float(np.linalg.norm(values - [1., 4., .6, .4]))},
                "first_contact_s": contact}}}
        return result

    monkeypatch.setattr(target_yaw, "evaluate_candidates", fake_evaluate)
    monkeypatch.setattr(target_yaw, "_residual",
                        lambda output, state, trial, run: profiles[run.parent.name] - [1., 4., .6, .4])
    original = {"yaw_torque_scale": .5, "linear_damping_angular": [1., 2., 3.]}
    selected, _, diagnostics = target_yaw.fit_target_yaw(
        tmp_path, state, original, "training", jobs=2, runs_dir=tmp_path / "runs", rounds=1,
    )
    assert "_step" not in selected
    assert diagnostics["selected_contact_count"] == 0


def test_target_fit_backtracks_when_large_steps_increase_error(tmp_path: Path, monkeypatch) -> None:
    (tmp_path / "vehicle.json").write_text(json.dumps({"thrusters": [{"name": "a", "kind": "horizontal"}]}))
    state = {"trials": {"training": {"split": "train", "path": "."}}}
    candidates = {}

    def fake_evaluate(output, state, profiles, trial_names, *, jobs, runs_dir):
        scores = {}
        for label, profile in profiles.items():
            candidates[label] = profile["yaw_torque_thruster_scales"]["a"]
            delta = candidates[label] - .5
            scores[label] = {"contact_count": 0, "trials": {"training": {
                "rmse": {"Yaw": abs(1. + delta + 50. * delta * delta)}, "first_contact_s": None}}}
        return scores

    def residual(output, state, trial, run):
        delta = candidates[run.parent.name] - .5
        return np.array([1. + delta + 50. * delta * delta])

    monkeypatch.setattr(target_yaw, "evaluate_candidates", fake_evaluate)
    monkeypatch.setattr(target_yaw, "_residual", residual)
    original = {"yaw_torque_scale": .5, "linear_damping_angular": [1., 2., 3.]}
    selected, _, diagnostics = target_yaw.fit_target_yaw(
        tmp_path, state, original, "training", jobs=2, runs_dir=tmp_path / "runs", rounds=1,
    )
    assert selected == "target_r0_step4"
    assert diagnostics["final_cost"] < diagnostics["initial_cost"]


def test_target_fit_keeps_improving_probe_if_all_search_steps_fail(tmp_path: Path, monkeypatch) -> None:
    (tmp_path / "vehicle.json").write_text(json.dumps({"thrusters": [{"name": "a", "kind": "horizontal"}]}))
    state = {"trials": {"training": {"split": "train", "path": "."}}}

    def fake_evaluate(output, state, profiles, trial_names, *, jobs, runs_dir):
        return {label: {"contact_count": 0, "trials": {"training": {
            "rmse": {"Yaw": .8 if label == "target_r0_fd0" else 2. if "_step" in label else 1.},
            "first_contact_s": None}}} for label in profiles}

    def residual(output, state, trial, run):
        label = run.parent.name
        return np.array([.8 if label == "target_r0_fd0" else 2. if "_step" in label else 1.])

    monkeypatch.setattr(target_yaw, "evaluate_candidates", fake_evaluate)
    monkeypatch.setattr(target_yaw, "_residual", residual)
    original = {"yaw_torque_scale": .5, "linear_damping_angular": [1., 2., 3.]}
    selected, fitted, diagnostics = target_yaw.fit_target_yaw(
        tmp_path, state, original, "training", jobs=2, runs_dir=tmp_path / "runs", rounds=1,
    )
    assert selected == "target_r0_fd0"
    assert fitted["yaw_torque_thruster_scales"]["a"] == pytest.approx(.52)
    assert diagnostics["final_cost"] < diagnostics["initial_cost"]


def test_warmup_uses_original_search_then_fine_search_even_if_warmup_fails(tmp_path: Path, monkeypatch) -> None:
    (tmp_path / "vehicle.json").write_text(json.dumps({"thrusters": [{"name": "a", "kind": "horizontal"}]}))
    state = {"trials": {"training": {"split": "train", "path": "."}}}
    profiles = {}

    def fake_evaluate(output, state, candidates, trial_names, *, jobs, runs_dir):
        result = {}
        for label, profile in candidates.items():
            profiles[label] = np.array([profile["yaw_torque_thruster_scales"]["a"],
                                        profile["linear_damping_angular"][2],
                                        profile["quadratic_damping_angular"][2], profile["yaw_damping_asymmetry"]])
            result[label] = {"contact_count": 0, "trials": {"training": {
                "rmse": {"Yaw": 1.0}, "first_contact_s": None}}}
        return result

    def residual(output, state, trial, run):
        label = run.parent.name
        if label.startswith("target_r0_step"):
            return np.full(4, 100.)
        return profiles[label] - [.3, 3.5, .5, .3]

    monkeypatch.setattr(target_yaw, "evaluate_candidates", fake_evaluate)
    monkeypatch.setattr(target_yaw, "_residual", residual)
    original = {"yaw_torque_scale": .5, "linear_damping_angular": [1., 2., 3.]}
    _, _, diagnostics = target_yaw.fit_target_yaw(
        tmp_path, state, original, "training", jobs=2, runs_dir=tmp_path / "runs", rounds=1, warmup=True,
    )
    assert diagnostics["rounds"][0]["phase"] == "warmup"
    assert diagnostics["rounds"][0]["accepted"] is None
    assert diagnostics["rounds"][1]["phase"] == "fine"
    assert diagnostics["rounds"][1]["accepted"] is not None
    assert "target_r0_fd2" not in profiles
    assert "target_r1_fd2" in profiles
    assert profiles["target_r0_fd0"][0] - profiles["target_seed"][0] == pytest.approx(.04)
    assert profiles["target_r0_fd1"][1] - profiles["target_seed"][1] == pytest.approx(.2)
    assert profiles["target_r0_fd3"][3] - profiles["target_seed"][3] == pytest.approx(.04)


def test_final_selection_guards_contacts_and_five_second_yaw() -> None:
    def row(yaw, short, contacts=0):
        return {"contact_count": contacts, "trials": {"training": {
            "rmse": {"Yaw": yaw}, "yaw_increment_rmse_deg": {"5": short},
        }}}

    scores = {
        "seed": row(100., 20.),
        "safe": row(50., 21.),
        "short_regression": row(20., 21.01),
        "contact": row(10., 19., 1),
    }
    selected, details = target_yaw.select_target_result(scores, "training", "seed")
    assert selected == "safe"
    assert details["candidates"]["short_regression"]["rejected_by"] == ["five_second_yaw"]
    assert details["candidates"]["contact"]["rejected_by"] == ["contact"]
