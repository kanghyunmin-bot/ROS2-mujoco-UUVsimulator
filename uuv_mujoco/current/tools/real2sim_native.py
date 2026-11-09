"""Evaluate candidate profiles using isolated production MuJoCo replays."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from tempfile import TemporaryDirectory

import numpy as np

from score_bluerov2_dfki import evaluate

# Dimensioned scales balance the optimization objective; they are not measured
# uncertainties or claims about the accuracy of the candidate.
AXIS_SCALES = {"Surge": .2, "Sway": .2, "Heave": 100., "Roll": 10., "Pitch": 10., "Yaw": 180.}
YAW_INCREMENT_SCALES = {1: 5., 5: 15., 10: 25.}
PROFILE_NAME = "real2sim_native_candidate"


def evaluate_candidates(
    output: Path,
    state: dict,
    profiles: dict[str, dict],
    trial_names: list[str],
    *,
    jobs: int,
    runs_dir: Path,
    duration_s: float | None = None,
) -> dict[str, dict]:
    """Score profiles on training trials with production physics and isolated processes.

    Args:
        output: Prepared output directory containing the scene, manifest and thrust table.
        state: Build state with trial metadata and relative scene path.
        profiles: Candidate labels and fully resolved profile dictionaries.
        trial_names: Training trial names to evaluate.
        jobs: Maximum number of independent replay processes.
        runs_dir: Parent directory for persistent ``label/trial`` replay outputs.
        duration_s: Optional maximum replay duration per trial [s]; None uses the entire run.

    Returns:
        Per-candidate normalized objective, contact count and per-trial scores.
    """
    output, runs_dir = Path(output).resolve(), Path(runs_dir).resolve()
    if not profiles or any(not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name)
                           for name in profiles):
        raise ValueError("Candidates need distinct safe labels")
    if not trial_names or len(trial_names) != len(set(trial_names)):
        raise ValueError("Select at least one distinct training trial")
    if not 1 <= jobs <= 32 or (duration_s is not None and (not math.isfinite(duration_s) or duration_s < 1)):
        raise ValueError("Native objective requires duration >= 1 s and 1-32 jobs")
    for name in trial_names:
        if (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name)
                or name not in state["trials"] or state["trials"][name].get("split") != "train"):
            raise ValueError(f"Native objective only accepts training trials: {name}")
    runs_dir.mkdir(parents=True, exist_ok=True)
    if any((runs_dir / label / name).exists() for label in profiles for name in trial_names):
        raise ValueError("Native replay output already exists")
    runner = Path(__file__).with_name("replay_bluerov2_dfki.py")

    profile_paths = {}
    for label, profile in profiles.items():
        candidate_dir = runs_dir / label
        candidate_dir.mkdir(parents=True, exist_ok=True)
        path = candidate_dir / "native_profile.json"
        path.write_text(json.dumps({PROFILE_NAME: profile}, allow_nan=False), encoding="utf-8")
        profile_paths[label] = path

    def run_trial(pair: tuple[str, str]) -> tuple[str, str, dict]:
        label, name = pair
        metadata = state["trials"][name]
        trial = (output / metadata["path"]).resolve()
        destination = runs_dir / label / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        command = [
            sys.executable, str(runner), "--pwm", str(trial / "pwm.csv"),
            "--real", str(trial / "real.csv"), "--voltage", str(trial / "voltage.csv"),
            "--depth-m", str(metadata["initial_depth_m"]),
            "--depth-reference", metadata["initial_depth_reference"],
            "--scene", str(output / state["scene"]), "--vehicle-manifest", str(output / "vehicle.json"),
            "--thruster-perf-file", str(output / "thruster_performance.json"),
            "--profile-file", str(profile_paths[label]), "--profile", PROFILE_NAME,
            "--output", str(destination),
        ]
        if duration_s is not None:
            command.extend(["--duration-s", str(duration_s)])
        completed = subprocess.run(command, capture_output=True, text=True)
        if completed.returncode:
            raise RuntimeError(f"Native replay failed for {label}/{name}: "
                               f"{completed.stderr[-3000:]} {completed.stdout[-3000:]}")
        scores, _ = evaluate(trial / "real.csv", [destination],
                             measured_max_gap_s=float(metadata.get("measured_max_gap_s", .04)))
        values = {metric["axis"]: metric["rmse"][destination.name] for metric in scores["metrics"]}
        contact = scores["runs"][destination.name].get("first_contact_s")
        increments = {str(seconds): scores["yaw_diagnostics"][key]["rmse_deg"][destination.name]
                      for seconds in YAW_INCREMENT_SCALES
                      if (key := f"increment_{seconds}s") in scores["yaw_diagnostics"]}
        normalized = [values[axis] / scale for axis, scale in AXIS_SCALES.items()]
        normalized.extend(increments[str(seconds)] / scale for seconds, scale in YAW_INCREMENT_SCALES.items()
                          if str(seconds) in increments)
        return label, name, {"objective": float(np.mean(normalized)), "rmse": values,
                             "yaw_increment_rmse_deg": increments,
                             "yaw_rate_bias_deg_s": scores["yaw_rate_bias_deg_s"][destination.name],
                             "first_contact_s": contact}

    pairs = [(label, trial) for label in profiles for trial in trial_names]
    with ThreadPoolExecutor(max_workers=min(jobs, len(pairs))) as executor:
        results = list(executor.map(run_trial, pairs))
    grouped = {label: {"trials": {}} for label in profiles}
    for label, name, result in results:
        grouped[label]["trials"][name] = result
    for data in grouped.values():
        trials = data["trials"].values()
        data.update(objective=float(np.mean([row["objective"] for row in trials])),
                    contact_count=sum(row["first_contact_s"] is not None for row in data["trials"].values()),
                    duration_s=duration_s, axis_scales=AXIS_SCALES.copy(),
                    yaw_increment_scales=YAW_INCREMENT_SCALES.copy())
    return grouped


def evaluate_candidate(output: Path, state: dict, profile: dict, trial_names: list[str], *, jobs: int = 2,
                       runs_dir: Path | None = None, duration_s: float | None = None) -> dict:
    """Score one fully resolved profile, optionally retaining production replays."""
    if runs_dir is not None:
        return evaluate_candidates(output, state, {"candidate": profile}, trial_names, jobs=jobs,
                                   runs_dir=runs_dir, duration_s=duration_s)["candidate"]
    with TemporaryDirectory(prefix="real2sim-native-runs-") as temp:
        return evaluate_candidates(output, state, {"candidate": profile}, trial_names, jobs=jobs,
                                   runs_dir=Path(temp), duration_s=duration_s)["candidate"]


def refinement_profiles(profiles: dict, scores: dict, incumbent: str) -> tuple[dict, dict]:
    """Propose local candidates using the measured response of the first native pass."""
    proposals = {}
    diagnostics = {}
    if "directional" in profiles and not scores["directional"]["contact_count"] and not scores["symmetric"]["contact_count"]:
        symmetric, directional = profiles["symmetric"], profiles["directional"]
        trials = scores["symmetric"]["trials"]
        initial = np.array([trials[name]["yaw_rate_bias_deg_s"] for name in trials])
        change = np.array([scores["directional"]["trials"][name]["yaw_rate_bias_deg_s"]
                           for name in trials]) - initial
        sensitivity = float(change @ change)
        if sensitivity > 1e-9:
            target = float(np.clip(-(initial @ change) / sensitivity, 0, 1))
            diagnostics["yaw_blend_from_native_bias"] = target
            for label, fraction in (("yaw_bias_step", target), ("yaw_half_step", .5 * target),
                                    ("yaw_quarter_step", .25 * target)):
                if not 0.02 <= fraction <= .98:
                    continue
                overrides = directional["yaw_torque_thruster_scales"]
                baseline = symmetric["yaw_torque_scale"]
                proposals[label] = {
                    **symmetric,
                    "yaw_torque_thruster_scales": {
                        motor: baseline + fraction * (gain - baseline) for motor, gain in overrides.items()
                    },
                    "yaw_damping_asymmetry": fraction * directional["yaw_damping_asymmetry"],
                }
    if incumbent != "initial" and {"buoyancy_lower", "buoyancy_higher"} <= profiles.keys():
        buoyancy = profiles[incumbent]["buoyancy_scale"]
        for label, delta in (("depth_half_lower", -.001), ("depth_half_higher", .001)):
            if .952 <= buoyancy + delta <= 1.048:
                proposals[label] = {**profiles[incumbent], "buoyancy_scale": buoyancy + delta}
    return proposals, diagnostics


def select_refinement(scores: dict, incumbent: str) -> tuple[str, dict]:
    """Accept a refinement only when native replay improves without material regression."""
    def metrics(score):
        trials = list(score["trials"].values())
        terms = {axis: float(np.mean([row["rmse"][axis] for row in trials])) for axis in AXIS_SCALES}
        for horizon in YAW_INCREMENT_SCALES:
            values = [row["yaw_increment_rmse_deg"][str(horizon)] for row in trials
                      if str(horizon) in row["yaw_increment_rmse_deg"]]
            if values:
                terms[f"yaw_{horizon}s"] = float(np.mean(values))
        terms["yaw_bias_rms"] = float(np.sqrt(np.mean([row["yaw_rate_bias_deg_s"] ** 2 for row in trials])))
        return terms

    baseline = metrics(scores[incumbent])
    assessed = {}
    for label, score in scores.items():
        values = metrics(score)
        reasons = []
        if score["contact_count"] > scores[incumbent]["contact_count"]:
            reasons.append("contacts")
        for axis, scale in AXIS_SCALES.items():
            if axis != "Yaw" and values[axis] > baseline[axis] + max(.03 * baseline[axis], .01 * scale):
                reasons.append(axis)
        for horizon, scale in YAW_INCREMENT_SCALES.items():
            key = f"yaw_{horizon}s"
            if key in baseline and values[key] > baseline[key] + max(.05 * baseline[key], .01 * scale):
                reasons.append(key)
        quality = score["objective"] + .25 * values["Yaw"] / 180 + .15 * values["yaw_bias_rms"] / 2
        assessed[label] = {"score": quality, "rejected_by": reasons, "metrics": values}
    accepted = [label for label in assessed if not assessed[label]["rejected_by"]]
    selected = min(accepted, key=lambda label: (scores[label]["contact_count"], assessed[label]["score"]))
    return selected, assessed
