#!/usr/bin/env python3
"""Fit an optional pressure observation envelope from a real bag export.

Fits adjacent-difference scatter [Pa] in a low-motion interval. This is not
an intrinsic sensor-noise or hydrodynamic calibration. Held-out samples are
used only for evaluation; source observations and default profiles stay intact.
"""

from __future__ import annotations

import argparse
from dataclasses import fields, replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from bridge.bar30_sensor_model import Bar30SensorConfig, Bar30SensorModel


def difference_scatter(values: np.ndarray) -> float:
    """Return adjacent-difference scatter [Pa] with sample variance correction."""
    values = np.asarray(values, dtype=float)
    if values.ndim != 1 or len(values) < 20 or not np.isfinite(values).all():
        raise ValueError("Need at least 20 finite scalar pressure observations")
    return float(np.std(np.diff(values), ddof=1) / np.sqrt(2.0))


def model_scatter(profile: dict, count: int, seeds: range = range(100)) -> np.ndarray:
    """Evaluate fixed-pressure observation scatter [Pa] across repeatable seeds."""
    names = {field.name for field in fields(Bar30SensorConfig)}
    cfg = Bar30SensorConfig(
        **{k: v for k, v in profile["bar30"]["model"].items() if k in names}
    )
    results = []
    for seed in seeds:
        model = Bar30SensorModel(replace(cfg, seed=seed))
        values = [
            model.sample(
                104_000.0, sample_time_s=j / cfg.nominal_rate_hz
            ).measured_pressure_pa
            for j in range(count)
        ]
        results.append(difference_scatter(np.array(values)))
    return np.array(results)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--numeric_npz", type=Path, required=True)
    parser.add_argument("--profile_json", type=Path, required=True)
    parser.add_argument("--source_manifest", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    args = parser.parse_args()
    source = json.loads(args.source_manifest.read_text())
    expected = "195aca87161fb180bc7c7ee0574523f7840399e439f7b1d245313d8d2faf2d52"
    if source.get("sha256") != expected:
        raise ValueError(
            "Fixed low-motion windows apply only to the reviewed April 2 bag"
        )
    topics = json.loads(args.profile_json.read_text())
    start = min(topic["start"] for topic in topics.values())
    with np.load(args.numeric_npz, allow_pickle=False) as arrays:
        pressure = arrays["__mavros__imu__static_pressure"]
    relative = pressure[:, 0] - start
    train_rows = pressure[(relative >= 90) & (relative < 115)]
    test_rows = pressure[(relative >= 115) & (relative < 140)]
    for rows in (train_rows, test_rows):
        if (
            len(rows) < 20
            or np.any(np.diff(rows[:, 0]) <= 0)
            or np.max(np.diff(rows[:, 0])) > 0.85
        ):
            raise ValueError(
                "Pressure fit requires ordered, densely sampled low-motion windows"
            )
    train = difference_scatter(train_rows[:, 1])
    held_out = difference_scatter(test_rows[:, 1])
    base_path = CURRENT / "config/sensor_models/imu_bar30_uncalibrated_prior.json"
    base = json.loads(base_path.read_text())
    tuned = json.loads(json.dumps(base))
    # Preserve uncertain quantization, drift, offset, thermal and IMU priors.
    quantization = base["bar30"]["model"]["quantization_pa"]
    fitted = float(np.sqrt(max(0.0, train**2 - quantization**2 / 12)))
    tuned["profile"] = "bag_20260402_pressure_observation_envelope"
    tuned["calibration_scope"] = (
        "partial_empirical_observation_envelope_not_hardware_calibration"
    )
    tuned["bar30"]["model"].update(white_noise_std_pa=fitted, nominal_rate_hz=2.0)
    tuned["bar30"]["timing"]["capture"]["rate_hz"] = 2.0
    tuned["bar30"]["model"]["basis"] = (
        "Bag 2026-04-02 low-motion 90-115 s adjacent pressure differences; "
        "2 Hz observed output envelope, not independently measured ADC capture rate. "
        "Only white scatter and effective schedule fitted; all other terms retain priors."
    )
    evidence = {
        "source_zip_sha256": source["sha256"],
        "numeric_npz_sha256": hashlib.sha256(args.numeric_npz.read_bytes()).hexdigest(),
        "fit_window_seconds": [90.0, 115.0],
        "held_out_window_seconds": [115.0, 140.0],
        "fit_n": len(train_rows),
        "held_out_n": len(test_rows),
        "fit_difference_scatter_pa": train,
        "held_out_difference_scatter_pa": held_out,
        "fitted_white_noise_std_pa": fitted,
        "limitations": [
            "One session; adjacent held-out window is not independent hardware validation.",
            "Residual water motion, vibration and frontend filtering remain confounded with noise.",
            "2 Hz models this recorded output deployment, not current desired 10 Hz operation.",
            "IMU, DVL, camera, clocks, mass, drag and buoyancy were not fitted.",
            "Do not use this sparse-output profile as a fix for VLA freshness failures.",
        ],
    }
    tuned["evidence"]["bag_20260402_fit"] = evidence
    baseline = json.loads(json.dumps(base))
    baseline["bar30"]["model"]["nominal_rate_hz"] = 2.0
    baseline["bar30"]["timing"]["capture"]["rate_hz"] = 2.0
    old = model_scatter(baseline, len(test_rows))
    new = model_scatter(tuned, len(test_rows))
    evidence["simulation_check"] = {
        "seeds": 100,
        "samples_per_seed": len(test_rows),
        "same_effective_rate_hz": 2.0,
        "baseline_scatter_median_pa": float(np.median(old)),
        "tuned_scatter_median_pa": float(np.median(new)),
        "baseline_abs_error_pa": float(abs(np.median(old) - held_out)),
        "tuned_abs_error_pa": float(abs(np.median(new) - held_out)),
        "tuned_p05_p95_pa": np.quantile(new, [0.05, 0.95]).tolist(),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "imu_bar30_bag_20260402.json").write_text(
        json.dumps(tuned, indent=2) + "\n"
    )
    (args.output_dir / "pressure_fit.json").write_text(
        json.dumps(evidence, indent=2) + "\n"
    )
    print(json.dumps(evidence, indent=2))


if __name__ == "__main__":
    main()
