"""Build and evaluate a MuJoCo vehicle package against recorded motor outputs."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import numpy as np

from replay_bluerov2_dfki import ANGLE, RATE, RUNTIME, VELOCITY, initial_state, read_rows
from replay_recorded_pwm_plant import sha256
from score_bluerov2_dfki import evaluate

SLUG = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")
CANDIDATE = "real2sim_candidate"
YAW_TARGET = "real2sim_yaw_target"


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def package_file(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if Path(relative).is_absolute() or root not in path.parents or not path.is_file():
        raise ValueError(f"Expected a file inside {root}: {relative}")
    return path


def validate_trial(path: Path, motor_count: int, min_duration_s: float) -> dict:
    real_path = package_file(path, "real.csv")
    pwm_path = package_file(path, "pwm.csv")
    voltage_path = package_file(path, "voltage.csv")
    meta = json.loads(package_file(path, "metadata.json").read_text(encoding="utf-8"))
    required = {"depth_m", *VELOCITY, *RATE, *ANGLE}
    real = read_rows(real_path)
    if not required.issubset(real.dtype.names or ()):
        raise ValueError(f"Missing measured fields in {real_path}: {sorted(required - set(real.dtype.names or ()))}")
    seed = initial_state(real)
    start = float(seed["t_s"])
    duration = real["t_s"][-1] - start
    if duration < min_duration_s:
        raise ValueError(f"Need at least {min_duration_s:g} s of measured response in {path}")
    pwm = read_rows(pwm_path)
    expected = tuple(f"pwm{i}" for i in range(1, motor_count + 1))
    if set(pwm.dtype.names or ()) != {"t_s", *expected}:
        raise ValueError(f"PWM fields in {path} must be t_s plus {', '.join(expected)}")
    pulses = np.column_stack([pwm[name] for name in expected])
    if not np.all((pulses >= 1100) & (pulses <= 1900) & (pulses == np.rint(pulses))):
        raise ValueError(f"PWM must contain integer values from 1100 to 1900 µs: {path}")
    if pwm["t_s"][0] > start + .25 or pwm["t_s"][-1] < real["t_s"][-1] - .25:
        raise ValueError(f"PWM does not cover measured interval (0.25 s tolerance): {path}")
    voltage = np.genfromtxt(voltage_path, delimiter=",", names=True, ndmin=1)
    if not {"sim_time", "voltage_v"}.issubset(voltage.dtype.names or ()) or len(voltage) < 2:
        raise ValueError(f"Voltage must contain sim_time,voltage_v: {path}")
    t, v = voltage["sim_time"], voltage["voltage_v"]
    if not (np.isfinite(t).all() and np.isfinite(v).all() and (v > 0).all()
            and abs(t[0]) <= 1e-9 and (np.diff(t) > 0).all()):
        raise ValueError(f"Voltage must be positive with increasing time from zero: {path}")
    if t[-1] < duration - .25 or (np.diff(t) > .25 + 1e-9).any():
        raise ValueError(f"Voltage does not cover measured interval (0.25 s tolerance): {path}")
    if start - real["t_s"][0] > 1e-8:
        raise ValueError(f"The first real.csv row must contain a complete initial state: {path}")
    depth = meta.get("initial_depth_m", meta.get("initial_pressure_depth_m"))
    reference = meta.get("initial_depth_reference", "pressure" if "initial_pressure_depth_m" in meta else "base_link")
    if not isinstance(depth, (float, int)) or not math.isfinite(depth) or depth <= 0:
        raise ValueError(f"Initial depth must be a positive number in {path}/metadata.json")
    if reference not in {"pressure", "base_link"}:
        raise ValueError(f"Invalid initial_depth_reference in {path}/metadata.json")
    return {"path": str(path), "initial_depth_m": depth, "initial_depth_reference": reference,
            "measured_max_gap_s": max(.04, min(.5, 2.5 * float(np.median(np.diff(real["t_s"]))))),
            "measured_sha256": sha256(real_path),
            "source": meta.get("source"), "source_files_sha256": meta.get("source_files_sha256")}


def read_package(root: Path) -> tuple[dict, Path, Path, dict, dict, dict]:
    sys.path.insert(0, str(RUNTIME))
    from physics.sim_profile_helpers import build_sim_profile, canonical_profile_name, load_sim_profiles
    from sim.runtime.vehicle_manifest import parse_thrusters

    root = root.resolve()
    manifest = json.loads(package_file(root, "vehicle.json").read_text(encoding="utf-8"))
    if manifest.get("version") != 1 or not SLUG.fullmatch(manifest.get("name", "")):
        raise ValueError("vehicle.json requires version 1 and a name using ASCII letters, digits, '_' or '-'")
    scene = package_file(root, manifest["scene"])
    if root / "model" not in scene.parents:
        raise ValueError("Put the main MJCF and its dependencies under model/")
    performance = package_file(root, manifest["thruster_perf_file"])
    profile_file = package_file(root, manifest["profile_file"])
    profiles = json.loads(profile_file.read_text(encoding="utf-8"))
    if not isinstance(profiles, dict) or any(not isinstance(profile, dict) for profile in profiles.values()):
        raise ValueError("Profile file must map profile names to objects")
    resolved_profiles, warning = load_sim_profiles(profile_file)
    if warning:
        raise ValueError(warning)
    profile_names = {canonical_profile_name(key) for key in profiles}
    if len(profile_names) != len(profiles) or {CANDIDATE, YAW_TARGET} & profile_names:
        raise ValueError("Profile file contains duplicate canonical names or a reserved name")
    _, _, motor_names, _ = parse_thrusters(manifest)
    if not all(SLUG.fullmatch(name) for name in motor_names):
        raise ValueError("Use ASCII letters, digits, '_' or '-' in thruster names")
    sensors = manifest.get("sensors", {})
    if not isinstance(sensors, dict):
        raise ValueError("sensors must be an object indexed by sensor type")
    refs = manifest.get("reference_profiles", {})
    if "reference_profile" in manifest:
        refs = {"reference": manifest["reference_profile"], **refs}
    if any(not SLUG.fullmatch(key) or key in {"nominal", "candidate", "yaw_target"} for key in refs):
        raise ValueError("Reference labels must be distinct from nominal and candidate")
    variants = {"nominal": manifest["base_profile"], "candidate": CANDIDATE, **refs}
    for key, value in variants.items():
        if key != "candidate":
            if canonical_profile_name(value) not in profile_names:
                raise ValueError(f"Profile must be declared in {profile_file}: {value}")
            build_sim_profile(resolved_profiles, value)
    initial_profile = build_sim_profile(resolved_profiles, manifest["base_profile"])
    seen = set()
    hashes = set()
    trials = {}
    for role in ("train", "holdout"):
        group = manifest[role]
        if not isinstance(group, list) or not group:
            raise ValueError(f"Declare at least one distinct {role} trial")
        for name in group:
            if not SLUG.fullmatch(name) or name in seen:
                raise ValueError(f"Invalid or repeated trial name: {name}")
            path = (root / "trials" / name).resolve()
            if root / "trials" not in path.parents:
                raise ValueError(f"Trial path escapes package: {path}")
            entry = validate_trial(path, len(motor_names), 3.1 if role == "train" else 1.0)
            if entry["measured_sha256"] in hashes:
                raise ValueError(f"Measured CSV is identical to another declared trial: {name}")
            if entry["initial_depth_reference"] == "pressure" and "pressure" not in sensors:
                raise ValueError(f"Pressure-referenced initial depth requires a pressure sensor: {name}")
            trials[name] = {"split": role, **entry}
            seen.add(name)
            hashes.add(entry["measured_sha256"])
    return manifest, scene, performance, profiles, initial_profile, {"variants": variants, "trials": trials}


def replay_command(output: Path, state: dict, trial: str, variant: str, destination: Path) -> list[str]:
    meta = state["trials"][trial]
    path = (output / meta["path"]).resolve()
    return [
        sys.executable, str(Path(__file__).with_name("replay_bluerov2_dfki.py")),
        "--pwm", str(path / "pwm.csv"), "--real", str(path / "real.csv"),
        "--voltage", str(path / "voltage.csv"), "--depth-m", str(meta["initial_depth_m"]),
        "--depth-reference", meta["initial_depth_reference"],
        "--output", str(destination), "--profile", state["variants"][variant],
        "--profile-file", str(output / "profiles.json"),
        "--thruster-perf-file", str(output / "thruster_performance.json"),
        "--scene", str(output / state["scene"]), "--vehicle-manifest", str(output / "vehicle.json"),
    ]


def run_replay(command: list[str], logfile: Path) -> None:
    logfile.parent.mkdir(parents=True, exist_ok=True)
    with logfile.open("w", encoding="utf-8") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        tail = "\n".join(logfile.read_text(encoding="utf-8", errors="replace").splitlines()[-12:])
        raise RuntimeError(f"Replay failed ({logfile})\n{tail}")


def score(output: Path, state: dict) -> dict:
    metrics = {}
    with (output / "rmse.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["split", "trial", "metric_group", "axis", "unit", "samples", *state["variants"]])
        for name, trial in state["trials"].items():
            base = output / "replays" / name
            scores, traces = evaluate(
                output / trial["path"] / "real.csv", [base / key for key in state["variants"]],
                measured_max_gap_s=trial["measured_max_gap_s"],
            )
            write_json(base / "metrics.json", scores)
            np.savetxt(base / "aligned.csv", np.column_stack(list(traces.values())), delimiter=",",
                       header=",".join(traces), comments="", fmt="%.9f")
            metrics[name] = scores
            for group, rows in (
                ("metrics", scores["metrics"]),
                ("kinematic_metrics", scores["kinematic_metrics"]),
                ("free_motion", scores["free_motion"]["metrics"]),
            ):
                for row in rows:
                    writer.writerow([trial["split"], name, group, row["axis"], row["unit"], row["samples"],
                                     *(row["rmse"][variant] for variant in state["variants"])])
    return metrics


def candidate_validation(comparison: dict) -> dict:
    contacts = {
        name: result["runs"]["candidate"]["first_contact_s"]
        for name, result in comparison.items()
        if result["runs"]["candidate"].get("first_contact_s") is not None
    }
    return {"candidate_requires_review": bool(contacts), "candidate_first_contact_s": contacts}


def fit_candidates(manifest, scene, initial_profile, performance, trial_dirs, max_evaluations):
    from real2sim_fit import fit_profile, refine_yaw_profile

    fitted, fit_report = fit_profile(manifest, scene, initial_profile, performance, trial_dirs, max_evaluations)
    directional, yaw_report = refine_yaw_profile(
        manifest, scene, fitted, performance, trial_dirs, max(1, max_evaluations // 2),
    )
    candidates = {"symmetric": fitted}
    if directional != fitted:
        candidates["directional"] = directional
    if "heave_mps" not in fit_report["weakly_excited_channels"]:
        for label, delta in (("buoyancy_lower", -.002), ("buoyancy_higher", .002)):
            buoyancy = fitted["buoyancy_scale"] + delta
            if .952 <= buoyancy <= 1.048:
                candidates[label] = {**fitted, "buoyancy_scale": buoyancy}
    return candidates, {**fit_report, "yaw_refinement": yaw_report}


def refine_training(output, state, candidates, training, trial_names, jobs, shortlist):
    from real2sim_native import evaluate_candidates, refinement_profiles, select_refinement

    incumbent = min(training, key=lambda label: (training[label]["contact_count"], training[label]["objective"]))
    proposed, diagnostics = refinement_profiles(candidates, training, incumbent)
    if not proposed:
        return incumbent, diagnostics
    print(f"Refining {incumbent} with {len(proposed)} candidates from the native training errors.", flush=True)
    followup = evaluate_candidates(output, state, proposed, trial_names, jobs=jobs, runs_dir=shortlist)
    selected, assessed = select_refinement({incumbent: training[incumbent], **followup}, incumbent)
    diagnostics.update(incumbent=incumbent, selected=selected, assessed=assessed)
    candidates.update(proposed)
    training.update(followup)
    return selected, diagnostics


def refine_target_trial(output, state, candidates, training, trial, jobs, runs_dir, rounds):
    from real2sim_target_yaw import fit_target_yaw

    if state["trials"].get(trial, {}).get("split") != "train":
        raise ValueError("The yaw target must be a declared training trial")
    seed = min(
        (label for label in candidates if label != "initial"),
        key=lambda label: (training[label]["trials"][trial]["first_contact_s"] is not None,
                           training[label]["trials"][trial]["rmse"]["Yaw"]),
    )
    print(f"Optimizing yaw on training trial {trial} from {seed} ({rounds} native rounds).", flush=True)
    label, profile, details = fit_target_yaw(
        output, state, candidates[seed], trial, jobs=jobs, runs_dir=runs_dir, rounds=rounds, warmup=True,
    )
    return label, profile, {**details, "seed": seed, "trial": trial}


def show(output: Path, trial: str | None, variant: str, rate: float, duration: float | None) -> None:
    if not math.isfinite(rate) or rate <= 0 or (duration is not None and (not math.isfinite(duration) or duration <= 0)):
        raise ValueError("Viewer playback rate and duration must be positive and finite")
    state = json.loads((output / "report.json").read_text(encoding="utf-8"))
    trial = trial or next(iter(state["trials"]))
    if trial not in state["trials"] or variant not in state["variants"]:
        raise ValueError(f"Select trial from {', '.join(state['trials'])} and variant from {', '.join(state['variants'])}")
    from tempfile import TemporaryDirectory

    with TemporaryDirectory(prefix="real2sim-view-") as temp:
        cmd = replay_command(output, state, trial, variant, Path(temp) / "viewer")
        cmd.extend(["--viewer", "--playback-rate", str(rate)])
        if duration is not None:
            cmd.extend(["--duration-s", str(duration)])
        subprocess.run(cmd, check=True)


def build(args: argparse.Namespace) -> None:
    from real2sim_native import evaluate_candidates
    from real2sim_scene import prepare_scene

    root = args.package.resolve()
    manifest, scene, performance, profiles, initial_profile, state = read_package(root)
    target = args.yaw_target_trial or manifest.get("yaw_target_trial")
    if target is not None and target not in manifest["train"]:
        raise ValueError("The yaw target must be a declared training trial")
    output = args.output.resolve()
    if root / "model" in output.parents:
        raise ValueError("Output cannot be inside the input model directory")
    output.mkdir(parents=True, exist_ok=False)
    scene, sensors = prepare_scene(scene, root / "model", manifest.get("sensors", {}), output / "model")
    state.update(vehicle=manifest["name"], scene=scene.relative_to(output).as_posix())
    for trial in state["trials"].values():
        trial["path"] = os.path.relpath(trial["path"], output)
    shutil.copy2(performance, output / "thruster_performance.json")
    write_json(output / "sensor_config.json", sensors)
    deployed = {
        **manifest,
        "scene": state["scene"],
        "profile_file": "profiles.json",
        "thruster_perf_file": "thruster_performance.json",
    }
    write_json(output / "vehicle.json", deployed)
    print(f"Prepared {scene}; fitting {len(manifest['train'])} training trial(s).", flush=True)
    candidates, fit_report = fit_candidates(
        manifest, scene, initial_profile,
        output / "thruster_performance.json",
        [(output / state["trials"][name]["path"]).resolve() for name in manifest["train"]], args.max_evaluations,
    )
    candidates["initial"] = initial_profile
    shortlist = output / "training_candidates"
    print(f"Selecting from {len(candidates)} profiles using full MuJoCo training replays.", flush=True)
    training = evaluate_candidates(output, state, candidates, manifest["train"], jobs=args.jobs, runs_dir=shortlist)
    selected, refinement = refine_training(
        output, state, candidates, training, manifest["train"], args.jobs, shortlist,
    )
    profiles[CANDIDATE] = candidates[selected]
    fit_report["native_selection"] = {
        "selected": selected, "training_only": True, "candidates": training, "refinement": refinement,
    }
    target_dir = output / "target_yaw_candidates"
    if target is not None:
        target_label, profiles[YAW_TARGET], fit_report["target_yaw"] = refine_target_trial(
            output, state, candidates, training, target, args.jobs, target_dir, args.yaw_target_rounds,
        )
        state["variants"]["yaw_target"] = YAW_TARGET
    write_json(output / "profiles.json", profiles)
    write_json(output / "fit.json", fit_report)
    print(f"Selected {selected}. Replaying the remaining training and untouched holdout comparisons.", flush=True)
    jobs = []
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        for name in state["trials"]:
            for variant in state["variants"]:
                base = output / "replays" / name
                if name in manifest["train"] and variant in {"nominal", "candidate"}:
                    label = selected if variant == "candidate" else "initial"
                    shutil.copytree(shortlist / label / name, base / variant)
                    continue
                if name == target and variant == "yaw_target":
                    shutil.copytree(target_dir / target_label / name, base / variant)
                    continue
                cmd = replay_command(output, state, name, variant, base / variant)
                jobs.append(executor.submit(run_replay, cmd, base / f"{variant}.log"))
        for job in jobs:
            job.result()
    comparison = score(output, state)
    validation = candidate_validation(comparison)
    write_json(output / "report.json", {**state, "fit": fit_report, "comparison": comparison, "validation": validation})
    print(f"Complete. Comparison: {output / 'rmse.csv'}", flush=True)
    if validation["candidate_requires_review"]:
        print("Candidate requires review due to simulated contact: "
              + ", ".join(validation["candidate_first_contact_s"]), flush=True)
    elif not args.headless and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        show(output, None, "candidate", 1.0, None)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="Prepare sensors, fit, and replay all trials")
    run.add_argument("package", type=Path)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--headless", action="store_true")
    run.add_argument("--jobs", type=int, default=2)
    run.add_argument("--max-evaluations", type=int, default=35)
    run.add_argument("--yaw-target-trial", help="Also fit a separate yaw profile on this training trial")
    run.add_argument("--yaw-target-rounds", type=int, default=3, choices=range(1, 5),
                     help="Number of fine yaw iterations after one warm-up")
    view = commands.add_parser("view", help="View a recorded trial with a generated or reference profile")
    view.add_argument("output", type=Path)
    view.add_argument("--trial")
    view.add_argument("--profile", default="candidate")
    view.add_argument("--playback-rate", type=float, default=1.0)
    view.add_argument("--duration-s", type=float)
    args = parser.parse_args()
    try:
        if args.command == "run":
            if not 1 <= args.jobs <= 32 or args.max_evaluations < 1:
                raise ValueError("--jobs must be 1–32 and --max-evaluations must be positive")
            build(args)
        else:
            show(args.output.resolve(), args.trial, args.profile, args.playback_rate, args.duration_s)
    except (KeyError, OSError, TypeError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f"real2sim: {exc}\n")


if __name__ == "__main__":
    main()
