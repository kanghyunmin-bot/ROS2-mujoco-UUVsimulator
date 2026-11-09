"""Bounded MuJoCo replay refinement for one explicitly selected training trial."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

from real2sim_native import evaluate_candidates
from score_bluerov2_dfki import evaluate


def _yaw_profile(template: dict, names: list[str], values: np.ndarray) -> dict:
    profile = copy.deepcopy(template)
    profile["yaw_torque_thruster_scales"] = dict(zip(names, values[:len(names)].tolist(), strict=True))
    if "linear_damping_diag" in profile:
        diagonal = list(profile["linear_damping_diag"])
        diagonal[5] = float(values[-3])
        profile["linear_damping_diag"] = diagonal
    angular = list(profile.get("linear_damping_angular", [0., 0., 0.]))
    angular[2] = float(values[-3])
    profile["linear_damping_angular"] = angular
    if "quadratic_damping_diag" in profile:
        diagonal = list(profile["quadratic_damping_diag"])
        diagonal[5] = float(values[-2])
        profile["quadratic_damping_diag"] = diagonal
    quadratic = list(profile.get("quadratic_damping_angular", [0., 0., 0.]))
    quadratic[2] = float(values[-2])
    profile["quadratic_damping_angular"] = quadratic
    profile["yaw_damping_asymmetry"] = float(values[-1])
    return profile


def _residual(output: Path, state: dict, trial: str, run: Path) -> np.ndarray:
    metadata = state["trials"][trial]
    measured = (output / metadata["path"] / "real.csv").resolve()
    _, trace = evaluate(measured, [run], measured_max_gap_s=float(metadata.get("measured_max_gap_s", .04)))
    yaw_valid = trace["yaw_valid"]
    yaw_error = trace[f"yaw_{trial}"] - trace["yaw_measured"]
    rate_error = trace[f"yaw_rate_{trial}"] - trace["yaw_rate_measured"]
    rate_valid = np.isfinite(rate_error)
    if yaw_valid.sum() < 10 or rate_valid.sum() < 10:
        raise ValueError(f"Insufficient valid yaw data in {trial}")
    parts = [yaw_error[yaw_valid] / (80. * np.sqrt(yaw_valid.sum()))]
    steps = 50  # The evaluator's common time grid is 10 Hz.
    common = yaw_valid[steps:] & yaw_valid[:-steps]
    if common.any():
        increment = yaw_error[steps:] - yaw_error[:-steps]
        parts.append(.35 * increment[common] / (20. * np.sqrt(common.sum())))
    parts.append(.10 * rate_error[rate_valid] / (15. * np.sqrt(rate_valid.sum())))
    return np.concatenate(parts)


def select_target_result(scores: dict, trial: str, baseline_label: str) -> tuple[str, dict]:
    """Select the lowest native yaw RMSE subject to seed-relative motion guards."""
    if baseline_label not in scores:
        raise ValueError("Target yaw selection requires the baseline replay")
    baseline = scores[baseline_label]["trials"][trial]
    baseline_short = baseline.get("yaw_increment_rmse_deg", {}).get("5")
    limit = float(baseline_short) * 1.05 if baseline_short is not None else None
    decisions = {}
    for label, score in scores.items():
        result = score["trials"][trial]
        yaw = float(result["rmse"]["Yaw"])
        short = result.get("yaw_increment_rmse_deg", {}).get("5")
        reasons = []
        if not math.isfinite(yaw):
            reasons.append("invalid_yaw")
        if score["contact_count"] > scores[baseline_label]["contact_count"]:
            reasons.append("contact")
        if limit is not None and (short is None or not math.isfinite(float(short)) or float(short) > limit + 1e-12):
            reasons.append("five_second_yaw")
        decisions[label] = {
            "yaw_rmse_deg": yaw,
            "five_second_yaw_rmse_deg": float(short) if short is not None else None,
            "contact_count": score["contact_count"],
            "rejected_by": reasons,
        }
    eligible = [label for label, result in decisions.items() if not result["rejected_by"]]
    if not eligible:
        raise ValueError("No target yaw replay satisfies the selection guards")
    selected = min(eligible, key=lambda label: decisions[label]["yaw_rmse_deg"])
    return selected, {
        "baseline": baseline_label,
        "five_second_yaw_limit_deg": limit,
        "selected": selected,
        "candidates": decisions,
    }


def fit_target_yaw(
    output: Path,
    state: dict,
    profile: dict,
    trial: str,
    *,
    jobs: int,
    runs_dir: Path,
    rounds: int,
    warmup: bool = False,
) -> tuple[str, dict, dict]:
    """Refine yaw dynamics on one declared training trial using native rollouts.

    The target trial must be declared as training. Each iteration estimates
    native rollout sensitivities, then tests bounded steps on that same
    trial. Other training runs and holdouts must be evaluated separately.

    Args:
        output: Prepared output folder containing the manifest and model.
        state: Build metadata with relative trial paths and split membership.
        profile: Starting resolved profile, which is never modified.
        trial: Name of the target training trial.
        jobs: Maximum number of simultaneous replay processes.
        runs_dir: Folder for all candidate profiles and replay evidence.
        rounds: Maximum number of fine finite-difference refinement iterations.
        warmup: Run the original six-parameter search before the fine iterations.

    Returns:
        Selected candidate label, profile, and JSON-compatible diagnostics.
    """
    output, runs_dir = Path(output).resolve(), Path(runs_dir).resolve()
    if trial not in state["trials"] or state["trials"][trial].get("split") != "train":
        raise ValueError("Target yaw refinement requires a declared training trial")
    if not 1 <= rounds <= 5:
        raise ValueError("Yaw refinement rounds must be between 1 and 5")
    manifest = json.loads((output / "vehicle.json").read_text())
    names = [item["name"] for item in manifest["thrusters"] if item.get("kind") == "horizontal"]
    if not names:
        raise ValueError("Target yaw refinement requires horizontal thrusters")
    overrides = profile.get("yaw_torque_thruster_scales", {})
    global_scale = float(profile.get("yaw_torque_scale", 1.))
    linear_yaw = (profile["linear_damping_diag"][5] if "linear_damping_diag" in profile
                  else profile.get("linear_damping_angular", [0., 0., 0.])[2])
    quadratic_yaw = (profile["quadratic_damping_diag"][5] if "quadratic_damping_diag" in profile
                     else profile.get("quadratic_damping_angular", [0., 0., 0.])[2])
    values = np.array([overrides.get(name, global_scale) for name in names]
                      + [linear_yaw, quadratic_yaw, profile.get("yaw_damping_asymmetry", 0.)], dtype=float)
    lower = np.r_[np.full(len(names), .05), .01, 0., -.6]
    upper = np.r_[np.full(len(names), 1.6), 8., 40., .6]
    if not np.isfinite(values).all() or np.any(values < lower) or np.any(values > upper):
        raise ValueError("Starting yaw parameters must lie inside the refinement bounds")
    scale = np.r_[np.full(len(names), .25), 1., 1., .2]
    difference = np.r_[np.full(len(names), .02), .1, .1, .02]

    label = "target_seed"
    current = _yaw_profile(profile, names, values)
    scored = evaluate_candidates(output, state, {label: current}, [trial], jobs=jobs, runs_dir=runs_dir)
    result_profiles = {label: current}
    result_scores = dict(scored)
    baseline = _residual(output, state, trial, runs_dir / label / trial)
    cost = float(baseline @ baseline / 2)
    initial_cost = cost
    rounds_report = []
    print(f"Target yaw {trial}: starting RMSE {scored[label]['trials'][trial]['rmse']['Yaw']:.2f} deg.", flush=True)
    total_rounds = rounds + int(warmup)
    for iteration in range(total_rounds):
        warmup_round = warmup and iteration == 0
        active = [index for index in range(len(values)) if not (warmup_round and index == len(names) + 1)]
        probe_sizes = (np.r_[np.full(len(names), .04), .2, .1, .04] if warmup_round else difference)
        print(f"Target yaw {trial}: evaluating refinement round {iteration + 1}/{total_rounds}.", flush=True)
        proposals = {}
        probe_values = {}
        steps = {}
        for index in active:
            value, delta = values[index], probe_sizes[index]
            signed = float(delta if value + delta <= upper[index] else -delta)
            probe = values.copy()
            probe[index] += signed
            probe_label = f"target_r{iteration}_fd{index}"
            proposals[probe_label] = _yaw_profile(profile, names, probe)
            probe_values[probe_label] = probe
            steps[index] = signed
        probe_scores = evaluate_candidates(output, state, proposals, [trial], jobs=jobs, runs_dir=runs_dir)
        columns = []
        rejected_probes = []
        for index, step in steps.items():
            probe_label = f"target_r{iteration}_fd{index}"
            row = _residual(output, state, trial, runs_dir / probe_label / trial)
            if row.shape != baseline.shape or not np.isfinite(row).all():
                raise ValueError("Target yaw replay has inconsistent valid samples")
            if probe_scores[probe_label]["contact_count"] > scored[label]["contact_count"]:
                rejected_probes.append(probe_label)
                columns.append(np.zeros_like(baseline))
            else:
                columns.append((row - baseline) / step)
        jacobian = np.column_stack(columns)
        # The bounded linear step uses scaled parameters and ridge damping to
        # avoid unstable updates when thruster effects are highly correlated.
        active_scale = scale[active]
        matrix = np.vstack((jacobian * active_scale, .03 * np.eye(len(active))))
        rhs = np.r_[-baseline, np.zeros(len(active))]
        lo = np.maximum(-1., (lower[active] - values[active]) / active_scale)
        hi = np.minimum(1., (upper[active] - values[active]) / active_scale)
        direction = np.zeros_like(values)
        direction[active] = lsq_linear(matrix, rhs, bounds=(lo, hi)).x * active_scale
        if float(np.linalg.norm(direction / scale)) < 1e-5:
            rounds_report.append({"round": iteration, "phase": "warmup" if warmup_round else "fine",
                                  "accepted": None, "reason": "small_step"})
            if warmup_round:
                print(f"Target yaw {trial}: warmup step is small; continuing with fine search.", flush=True)
                continue
            print(f"Target yaw {trial}: stopping because the estimated step is small.", flush=True)
            break
        candidates = {}
        candidate_values = {}
        step_sizes = (1., .5) if warmup_round else (1., .5, .25, .125, .0625)
        for step_index, step_scale in enumerate(step_sizes):
            candidate_label = f"target_r{iteration}_step{step_index}"
            next_values = np.clip(values + step_scale * direction, lower, upper)
            candidates[candidate_label] = _yaw_profile(profile, names, next_values)
            candidate_values[candidate_label] = next_values
        steps_scores = evaluate_candidates(output, state, candidates, [trial], jobs=jobs, runs_dir=runs_dir)
        evaluated_profiles = {**({} if warmup_round else proposals), **candidates}
        evaluated_values = {**({} if warmup_round else probe_values), **candidate_values}
        evaluated_scores = {**({} if warmup_round else probe_scores), **steps_scores}
        result_profiles.update(evaluated_profiles)
        result_scores.update(evaluated_scores)
        choices = {}
        for candidate_label in evaluated_profiles:
            residual = _residual(output, state, trial, runs_dir / candidate_label / trial)
            if residual.shape != baseline.shape or not np.isfinite(residual).all():
                raise ValueError("Target yaw replay has inconsistent valid samples")
            candidate_cost = float(residual @ residual / 2)
            choices[candidate_label] = {
                "cost": candidate_cost,
                "yaw_rmse_deg": evaluated_scores[candidate_label]["trials"][trial]["rmse"]["Yaw"],
                "first_contact_s": evaluated_scores[candidate_label]["trials"][trial]["first_contact_s"],
            }
        previous_contacts = scored[label]["contact_count"]
        accepted = [name for name, item in choices.items()
                    if item["first_contact_s"] is None or previous_contacts]
        accepted = [name for name in accepted if choices[name]["cost"] < cost - 1e-9]
        selected = min(accepted, key=lambda name: choices[name]["cost"]) if accepted else None
        rounds_report.append({
            "round": iteration, "phase": "warmup" if warmup_round else "fine",
            "cost_before": cost, "jacobian_rank": int(np.linalg.matrix_rank(jacobian * active_scale)),
            "contact_rejected_probes": rejected_probes,
            "probe_costs": {name: probe_scores[name]["trials"][trial]["rmse"]["Yaw"] for name in proposals},
            "choices": choices, "accepted": selected,
        })
        if selected is None:
            if warmup_round:
                print(f"Target yaw {trial}: no improving warmup step; continuing with fine search.", flush=True)
                continue
            print(f"Target yaw {trial}: no safe improving step in round {iteration + 1}.", flush=True)
            break
        label, current, values = selected, evaluated_profiles[selected], evaluated_values[selected]
        scored[label] = evaluated_scores[label]
        baseline = _residual(output, state, trial, runs_dir / label / trial)
        cost = float(baseline @ baseline / 2)
        print(f"Target yaw {trial}: accepted RMSE {scored[label]['trials'][trial]['rmse']['Yaw']:.2f} deg.",
              flush=True)

    label, selection = select_target_result(result_scores, trial, "target_seed")
    current = result_profiles[label]
    cost_residual = _residual(output, state, trial, runs_dir / label / trial)
    cost = float(cost_residual @ cost_residual / 2)
    diagnostics = {
        "method": "bounded finite-difference Gauss-Newton on production MuJoCo replays",
        "training_trial": trial,
        "parameters": [*[f"yaw_scale_{name}" for name in names], "linear_yaw_damping",
                       "quadratic_yaw_damping", "yaw_damping_asymmetry"],
        "initial_cost": initial_cost,
        "final_cost": cost,
        "selected": label,
        "rounds": rounds_report,
        "selected_yaw_rmse_deg": result_scores[label]["trials"][trial]["rmse"]["Yaw"],
        "selected_contact_count": result_scores[label]["contact_count"],
        "final_selection": selection,
        "objective": "10 Hz unwrapped yaw, 5 s yaw increments and yaw rate",
    }
    return label, current, diagnostics
