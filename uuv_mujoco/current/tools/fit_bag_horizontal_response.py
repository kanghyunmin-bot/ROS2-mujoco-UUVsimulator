"""Integral constrained least squares; coefficients are conditional effective parameters."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear
from scipy.signal import savgol_filter

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output_dir", type=Path, required=True)
parser.add_argument("--numeric_npz", type=Path, required=True)
parser.add_argument(
    "--profile_json",
    type=Path,
    required=True,
    help="Real topic metadata and record-time origin",
)
parser.add_argument("--sim_profiles", type=Path, required=True)
parser.add_argument("--sim_profile", default="bag0402_clearance")
parser.add_argument("--scene_xml", type=Path, required=True)
parser.add_argument("--thruster_voltage_trace", type=Path)
parser.add_argument(
    "--voltage_trace_bag_start_s",
    type=float,
    help="Bag receipt time [s] corresponding to voltage CSV time zero",
)
args = parser.parse_args()
if (args.thruster_voltage_trace is None) != (args.voltage_trace_bag_start_s is None):
    parser.error("voltage trace and its bag start time must be specified together")
R = args.output_dir.resolve()
R.mkdir(parents=True, exist_ok=True)
root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(root / "uuv_mujoco/current"))
import xml.etree.ElementTree as ET

from physics.sim_profile_helpers import load_sim_profiles
from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_SERVO_MAP as names,
)
from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS as signs,
)
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics
from sim.physics.thruster_force_performance import pwm_to_force_from_performance
from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.physics.thruster_voltage import load_voltage_trace, update_supply_voltage

n = np.load(args.numeric_npz)
prof = json.loads((args.profile_json).read_text())
t0 = min(v["start"] for v in prof.values())
d = n["__dvl__data"]
imu = n["__mavros__imu__data"]
rc = n["__mavros__rc__out"]
p, _ = load_sim_profiles(args.sim_profiles)
hyd = DistributedHullHydrodynamics.from_profile(p[args.sim_profile])
curve = load_thruster_performance_config(
    root / "uuv_mujoco/current/config/thruster_performance.json",
    requested_voltage=float(p[args.sim_profile]["thruster_voltage"]),
    direct=True,
)
voltage_model = {"mode": "constant_assumption", "voltage_v": curve["selected_voltage"]}
if args.thruster_voltage_trace is not None:
    trace = load_voltage_trace(
        args.thruster_voltage_trace, curve, time_offset_s=args.voltage_trace_bag_start_s
    )
    curve["voltage_trace"] = trace
    voltage_model = {
        "mode": "recorded_trace",
        "voltage_reference": trace["voltage_reference"],
        "csv_sha256": trace["csv_sha256"],
        "provenance": trace["provenance"],
        "bag_start_s": trace["time_offset_s"],
        "outside_trace": trace["outside_trace"],
    }
x = ET.parse(args.scene_xml)
gears = {
    e.get("name"): np.fromstring(e.get("gear"), sep=" ")[:3]
    for e in x.findall("./actuator/motor")
}
t = np.arange(19, 72, 0.1)
v = np.column_stack([np.interp(t, d[:, 0] - t0, d[:, j]) for j in (1, 2, 3)])
# Raw x is forward in both disputed DVL conventions. Surge-only fit avoids selecting lateral sign from fit quality.
u = savgol_filter(v[:, 0], 9, 2)
omega = np.column_stack([np.interp(t, imu[:, 0] - t0, imu[:, j]) for j in (5, 6, 7)])
pwm = np.column_stack([np.interp(t, rc[:, 0] - t0, rc[:, j]) for j in range(1, 9)])
forces = []
for bag_time, row in zip(t, pwm):
    update_supply_voltage(curve, float(bag_time))
    forces.append(
        [
            sum(
                pwm_to_force_from_performance((row[j] - 1500) / 400 * signs[j], curve)
                * gears[names[j]][a]
                for j in range(4)
            )
            for a in (0, 1)
        ]
    )
f = np.array(forces)


# Evaluate nominal fully-submerged straight-ahead distributed drag, no current.
def drag(speed):
    r = hyd.evaluate(
        body_position_world_m=np.array([0, 0, -0.6]),
        rotation_world_from_body=np.eye(3),
        linear_velocity_world_mps=np.array([speed, 0, 0]),
        angular_velocity_world_radps=np.zeros(3),
        current_world_mps=np.zeros(3),
        surface_height_world_m=0,
        time_s=0,
    )
    return float(r.force_world_n[0])


q0 = -drag(1.0)
mass = sum(float(c["mass"]) for c in p[args.sim_profile]["body_components"]) + float(
    p[args.sim_profile]["hydrodynamic_matrices"]["added_mass_6x6"][0][0]
)
# Nonoverlapping 1 s integration windows; no differentiated noisy target. Straight segments only.
rows = []
for start in np.arange(20, 71, 1.0):
    # Include interpolation and smoothing support, not only grid sample validity.
    if np.any(
        (d[:, 5] < 0.5)
        & (d[:, 0] - t0 >= start - 0.55)
        & (d[:, 0] - t0 <= start + 1.55)
    ):
        continue
    ids = np.where((t >= start - 1e-6) & (t <= start + 1 + 1e-6))[0]
    if len(ids) < 10:
        continue
    if np.max(abs(omega[ids, 2])) > 0.15 or np.max(abs(v[ids, 1])) > 0.06:
        continue
    # Exclude invalid DVL or >0.3 s nearest sample gaps throughout a window.
    di = np.searchsorted(d[:, 0] - t0, t[ids], side="right") - 1
    if np.any(d[di, 5] < 0.5) or np.any(t[ids] - (d[di, 0] - t0) > 0.3):
        continue
    dt = t[ids]

    def integ(z, sample_times=dt):
        return float(np.trapezoid(z, sample_times))

    rows.append(
        [
            start,
            mass * (u[ids[-1]] - u[ids[0]]),
            integ(f[ids, 0]),
            -integ(u[ids]),
            -integ(u[ids] * abs(u[ids])),
        ]
    )
a = np.array(rows)
if len(a) < 5:
    raise ValueError("Not enough valid straight integration windows")
np.savez(
    R / "fit_inputs.npz",
    t=t,
    v=v,
    u=u,
    omega=omega,
    pwm=pwm,
    nominal_force=f,
    windows=a,
)
train = a[:, 0] < 45
test = a[:, 0] >= 45
if sum(train) < 3 or sum(test) < 3:
    raise ValueError("Need at least three train and three held-out windows")
X = a[:, 2:5]
y = a[:, 1]
if not np.all(np.isfinite(a)):
    raise ValueError("Nonfinite identification inputs")
res = {}
scale = np.linalg.norm(X[train], axis=0)
if np.any(scale < 1e-10):
    raise ValueError("Insufficient excitation for thrust/drag identification")
sv = np.linalg.svd(X[train] / scale, compute_uv=False)
for label, cols, lo, hi in [
    ("thrust_linear_quadratic", [0, 1, 2], [0.01, 0, 0], [1.5, 200, 1000]),
    ("thrust_quadratic", [0, 2], [0.01, 0], [1.5, 1000]),
    ("quadratic_only", [2], [0], [1000]),
]:
    xx = X[:, cols]
    yy = y.copy()
    if label == "quadratic_only":
        yy -= X[:, 0]
    sc = np.linalg.norm(xx[train], axis=0)
    sol = lsq_linear(
        xx[train] / sc, yy[train], bounds=(np.array(lo) * sc, np.array(hi) * sc)
    )
    coef = sol.x / sc
    pred = xx @ coef + (X[:, 0] if label == "quadratic_only" else 0)
    res[label] = {
        "coefficients": coef.tolist(),
        "train_impulse_rmse_Ns": float(np.sqrt(np.mean((pred[train] - y[train]) ** 2))),
        "heldout_impulse_rmse_Ns": float(np.sqrt(np.mean((pred[test] - y[test]) ** 2))),
        "at_bound": sol.active_mask.tolist(),
    }
# Block bootstrap 1-second windows, train only. Identify instability, not independent real validation.
rng = np.random.default_rng(20260912)
boots = []
for _ in range(200):
    ix = rng.choice(np.where(train)[0], sum(train), replace=True)
    xx = X[ix][:, [0, 2]]
    sc = np.maximum(np.linalg.norm(xx, axis=0), 1e-12)
    z = lsq_linear(
        xx / sc, y[ix], bounds=(np.array([0.01, 0]) * sc, np.array([1.5, 1000]) * sc)
    )
    boots.append(z.x / sc)
report = {
    "calibration_status": "single_bag_effective_not_physical",
    "voltage_model": voltage_model,
    "mass_plus_added_mass_assumed_kg": mass,
    "nominal_surge_quadratic_drag_N_s2_m2": q0,
    "train_windows": a[train, 0].tolist(),
    "heldout_windows": a[test, 0].tolist(),
    "normalized_design_singular_values": sv.tolist(),
    "normalized_condition_number": float(sv[0] / sv[-1]),
    "models": res,
    "thrust_quadratic_bootstrap_p05_p50_p95": np.quantile(
        boots, [0.05, 0.5, 0.95], axis=0
    ).tolist(),
    "baseline_train_impulse_rmse_Ns": float(
        np.sqrt(np.mean((X[train, 0] + q0 * X[train, 2] - y[train]) ** 2))
    ),
    "baseline_heldout_impulse_rmse_Ns": float(
        np.sqrt(np.mean((X[test, 0] + q0 * X[test, 2] - y[test]) ** 2))
    ),
    "scope": "Surge only, effective coefficients conditional on mass and the recorded voltage_model assumption, using receipt-time motor feedback. Sway not identified; weak lateral excitation and coordinate ambiguity. July parameters not proven April settings. Voltage input alone does not identify physical thrust separately from hull drag.",
}
(R / "fit_result.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
