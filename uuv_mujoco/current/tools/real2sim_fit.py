"""Fit a bounded effective underwater model from recorded motor and motion data.

The numerical search uses a 50 Hz submerged rigid-body approximation.  Always
assess the resulting profile by replaying the same data in the MuJoCo runtime.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.linalg import qr
from scipy.spatial.transform import Rotation

from replay_bluerov2_dfki import ANGLE, AXES, RATE, RUNTIME, VELOCITY, read_rows
from rosbag_trend_math import interpolate_valid

sys.path.insert(0, str(RUNTIME))
from sim.physics.thruster_dynamics import thruster_response
from sim.physics.thruster_voltage import measured_voltage_surface, select_supply_voltage

try:
    from numba import njit
except ImportError:
    def njit(function):
        return function

DT = 0.02
RATE_SCALE = np.array([.18, .09, .10, .13, .11, .32])
POSE_SCALE = np.array([.7, .11, .12, .9])


def _geometry(scene_path: Path, thrusters: list[dict]) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    body = model.body("base_link")
    body_id = body.id
    if model.body_parentid[body_id] != 0:
        raise ValueError("base_link must be a direct child of world")
    joint = model.joint("world_joint")
    if (int(model.body_jntnum[body_id]) != 1 or int(model.body_jntadr[body_id]) != joint.id
            or int(model.jnt_type[joint.id]) != int(mujoco.mjtJoint.mjJNT_FREE)):
        raise ValueError("base_link must have exactly one free joint named world_joint")
    descendants = []
    for index in range(model.nbody):
        parent = int(model.body_parentid[index])
        while parent and parent != body_id:
            parent = int(model.body_parentid[parent])
        if index != body_id and parent == body_id:
            descendants.append(index)
    child_mass = float(np.sum(model.body_mass[descendants]))
    if child_mass > .001 * float(body.mass[0]):
        raise ValueError("the coarse fitter requires vehicle descendant mass below 0.1% of base_link mass")
    # The coarse integrator assumes diagonal inertia aligned with body axes.
    inertial_quat = np.asarray(body.iquat)
    if not np.allclose(np.abs(inertial_quat), [1, 0, 0, 0], atol=1e-7):
        raise ValueError("The coarse fitter requires body-aligned diagonal inertia")
    mass = float(body.mass[0])
    inertia = np.asarray(body.inertia).copy()
    com = np.asarray(body.ipos).copy()
    if mass <= 0 or np.any(inertia <= 0):
        raise ValueError("base_link mass and all diagonal inertias must be positive")
    rows = []
    for item in thrusters:
        name = item["name"]
        site = model.site("thr_" + name)
        motor = model.actuator(name)
        if model.site_bodyid[site.id] != body_id:
            raise ValueError(f"thr_{name} must be directly attached to base_link")
        if int(model.actuator_trnid[motor.id, 0]) != site.id:
            raise ValueError(f"{name} must actuate thr_{name}")
        gear = np.asarray(motor.gear)
        if not np.allclose(gear[3:], 0) or not np.isclose(np.linalg.norm(gear[:3]), 1):
            raise ValueError(f"{name} needs a unit translational gear and no direct torque")
        q = np.asarray(site.quat)
        direction = Rotation.from_quat([q[1], q[2], q[3], q[0]]).apply(gear[:3])
        lever = np.asarray(site.pos) - com
        rows.append(np.r_[direction * AXES, np.cross(lever, direction) * AXES])
    return np.asarray(rows), mass, inertia, com * AXES


def _read_trial(path: Path, surface: dict, thrusters: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    real, pwm = (read_rows(path / f"{name}.csv") for name in ("real", "pwm"))
    voltage = np.genfromtxt(path / "voltage.csv", delimiter=",", names=True, ndmin=1)
    if (len(voltage) < 2 or not np.isfinite(voltage["sim_time"]).all()
            or not np.isfinite(voltage["voltage_v"]).all()
            or abs(voltage["sim_time"][0]) > 1e-9 or np.any(np.diff(voltage["sim_time"]) <= 0)):
        raise ValueError(f"{path}: voltage trace must start at zero and have increasing finite timestamps")
    start = float(real["t_s"][0])
    time = np.arange(start, float(real["t_s"][-1]), DT)
    if len(time) < round(3 / DT):
        raise ValueError(f"{path}: trial must cover at least three seconds")
    max_gap = max(.04, min(.5, 2.5 * float(np.median(np.diff(real["t_s"])))))
    channels = []
    for field in (*VELOCITY, *RATE, "depth_m", *ANGLE):
        source = real[field].copy()
        if field == "yaw_deg":
            # Interpolating wrapped headings directly creates an artificial
            # sweep through zero at each transition across ±180 degrees.
            valid = np.isfinite(source)
            source[valid] = np.rad2deg(np.unwrap(np.deg2rad(source[valid])))
        channels.append(interpolate_valid(real["t_s"], source, time, max_gap_s=max_gap))
    measured = np.column_stack(channels)
    measured[:, 7:10] = np.deg2rad(measured[:, 7:10])
    packet = np.searchsorted(pwm["t_s"], time + 1e-7, side="right") - 1
    pwms = np.column_stack([pwm[f"pwm{i}"][packet.clip(min=0)] for i in range(1, len(thrusters) + 1)])
    if not np.isfinite(pwms).all() or np.any((pwms < 1100) | (pwms > 1900)):
        raise ValueError(f"{path}: PWM must be finite and within 1100--1900 us")
    pwms[packet < 0] = 1500
    volts = np.interp(time - start, voltage["sim_time"], voltage["voltage_v"])
    rpm = np.zeros(len(thrusters))
    force = np.zeros_like(pwms)
    signs = np.asarray([item.get("pwm_sign", 1) * item.get("forward_sign", 1) for item in thrusters])
    forward = np.asarray([item.get("forward_sign", 1) for item in thrusters])
    if np.any(np.abs(signs) != 1) or np.any(np.abs(forward) != 1):
        raise ValueError("motor polarity values must be +1 or -1")
    curve_pulses = 1500 + (pwms - 1500) * signs
    for i, (row, value) in enumerate(zip(curve_pulses, volts, strict=True)):
        select_supply_voltage(surface, float(value))
        target = np.interp(row, surface["pwm"], surface["rpm"])
        target[np.abs(row - 1500) <= 25] = 0.0
        rpm = np.array([thruster_response(a, b, DT, .04, .06) for a, b in zip(rpm, target, strict=True)])
        force[i] = np.interp(rpm, surface["rotor_rpm_knots"], surface["rotor_force_knots"]) * forward
    return measured, force


@njit
def _simulate(initial, thrust, parameters, mass, inertia, com, yaw_asymmetry=0.0):
    x = initial.copy()
    out = np.empty((len(thrust), 10))
    damping_linear = parameters[:6]
    damping_quadratic = parameters[6:12]
    weight = mass * 9.81
    buoyancy = weight * (1.0 + parameters[12] / 100)
    cob_x, cob_z = parameters[13] / 100, -parameters[14] / 100
    cob_y = -parameters[-2] / 100
    # measured twist is at the base origin, integration is at the mass center.
    x[0] += x[4] * com[2] - x[5] * com[1]
    x[1] += x[5] * com[0] - x[3] * com[2]
    x[2] += x[3] * com[1] - x[4] * com[0]
    sr0, cr0 = math.sin(initial[7]), math.cos(initial[7])
    st0, ct0 = math.sin(initial[8]), math.cos(initial[8])
    com_down_initial = -st0 * com[0] + sr0 * ct0 * com[1] + cr0 * ct0 * com[2]
    start_depth = initial[6]
    for i in range(len(thrust)):
        u, v, w, p, q, r, depth, roll, pitch, yaw = x
        out[i] = x
        out[i, 0] -= q * com[2] - r * com[1]
        out[i, 1] -= r * com[0] - p * com[2]
        out[i, 2] -= p * com[1] - q * com[0]
        sr, cr, st, ct = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
        ct = max(.06, ct)
        down_x, down_y, down_z = -st, sr * ct, cr * ct
        net = weight - buoyancy
        damping = damping_linear + damping_quadratic * np.abs(x[:6])
        # FLU yaw is the negative of FRD yaw. The runtime increases linear
        # damping for positive FRD yaw when the asymmetry is positive.
        damping[5] += damping_linear[5] * np.sign(r) * yaw_asymmetry
        wrench = thrust[i]
        x[0] += DT * ((wrench[0] + net * down_x) / mass - q * w + r * v)
        x[1] += DT * ((wrench[1] + net * down_y) / mass - r * u + p * w)
        x[2] += DT * ((wrench[2] + net * down_z) / mass - p * v + q * u)
        x[3] += DT * ((wrench[3] + buoyancy * (cob_z * down_y - cob_y * down_z)
                       + (inertia[1] - inertia[2]) * q * r) / inertia[0])
        x[4] += DT * ((wrench[4] - buoyancy * (cob_z * down_x - cob_x * down_z)
                       - parameters[-1] * u * abs(u) + (inertia[2] - inertia[0]) * p * r) / inertia[1])
        x[5] += DT * ((wrench[5] - buoyancy * (cob_x * down_y - cob_y * down_x)
                       + (inertia[0] - inertia[1]) * p * q) / inertia[2])
        x[:3] /= 1 + DT * damping[:3] / mass
        x[3:6] /= 1 + DT * damping[3:6] / inertia
        x[6] += DT * (u * down_x + v * down_y + w * down_z)
        out[i, 6] = depth - (down_x * com[0] + down_y * com[1] + down_z * com[2] - com_down_initial)
        yaw_rate = (sr * q + cr * r) / ct
        x[7] += DT * (p + st * yaw_rate)
        x[8] += DT * (cr * q - sr * r)
        x[9] += DT * yaw_rate
        if np.max(np.abs(x[:6])) > 6 or abs(x[8]) > 1.50 or abs(x[6] - start_depth) > 35:
            out[i + 1:] = out[i]
            break
    return out



def _motor_wrench(motors: np.ndarray, geometry: np.ndarray, gains: np.ndarray,
                  thrusters: list[dict], global_yaw_scale: float,
                  yaw_overrides: dict[str, float]) -> np.ndarray:
    """Convert motor thrusts to an FRD wrench with native yaw-only overrides."""
    force = motors * gains
    wrench = force @ geometry
    # Native yaw scaling applies only to horizontal actuators. Vertical
    # actuators retain their geometric yaw moment even when tilted.
    yaw_scales = np.array([
        float(yaw_overrides.get(item["name"], global_yaw_scale))
        if item.get("kind", "horizontal") == "horizontal" else 1.0
        for item in thrusters
    ])
    wrench[:, 5] = force @ (geometry[:, 5] * yaw_scales)
    return wrench


def _windows(measured: np.ndarray, duration: float, stride: float) -> list[tuple[int, int]]:
    result = []
    for i in range(0, len(measured) - round(3 / DT), round(stride / DT)):
        seed = np.flatnonzero(np.isfinite(measured[i:i + 50]).all(axis=1))
        if seed.size:
            start = i + int(seed[0])
            result.append((start, min(start + round(duration / DT), len(measured))))
    return result


def fit_profile(manifest: dict, scene_path: Path, initial_profile: dict,
                thruster_perf_file: Path, trial_dirs: list[Path], max_evaluations: int) -> tuple[dict, dict]:
    """Estimate an effective legacy damping/hydrostatic profile from training runs.

    The input profile must already have its ``extends`` chain resolved. The
    returned profile is a fresh dictionary suitable for replay and never
    changes the input. Validation and test runs must be excluded from trials.
    """
    if not trial_dirs or max_evaluations < 1:
        raise ValueError("at least one training trial and a positive evaluation budget are required")
    thrusters = manifest["thrusters"]
    if not thrusters:
        raise ValueError("at least one thruster is required")
    if initial_profile.get("thruster_response_model", "rpm") != "rpm":
        raise ValueError("the coarse fitter requires the RPM response model")
    for key in ("hydrodynamic_matrices", "distributed_hydrodynamics", "fossen_residual", "hydrostatic_restoring"):
        if initial_profile.get(key, {}).get("active", False):
            raise ValueError(f"the coarse fitter requires {key}.active to be false")
    if any(np.any(np.asarray(initial_profile.get(key, [0, 0, 0]), dtype=float))
           for key in ("added_mass_linear", "added_mass_angular")):
        raise ValueError("the coarse fitter requires zero added mass")
    if np.any(np.asarray(initial_profile.get("current_world", [0, 0, 0]), dtype=float)):
        raise ValueError("the coarse fitter requires still water")
    if initial_profile.get("yaw_torque_thruster_scales") or initial_profile.get("yaw_damping_asymmetry", 0):
        raise ValueError("use a symmetric yaw base profile; directional yaw settings can be replayed as references")
    geometry, mass, inertia, com = _geometry(Path(scene_path), thrusters)
    horizontal_yaw_levers = geometry[:, 5] * np.asarray([item["kind"] == "horizontal" for item in thrusters])
    yaw_from_horizontal_only = np.array_equal(horizontal_yaw_levers, geometry[:, 5])
    surface = measured_voltage_surface(json.loads(Path(thruster_perf_file).read_text()), include_rpm=True)
    trials = [_read_trial(Path(path), surface, thrusters) for path in trial_dirs]
    schedules = [(_windows(measured, 8, 8), _windows(measured, 60, 50)) for measured, _ in trials]
    if any(not short for short, _ in schedules):
        raise ValueError("every training run needs a complete 10-channel initial state and at least 3 s of data")
    names = [item["name"] for item in thrusters]
    gains = initial_profile.get("thruster_direct_gain_scales", {})
    initial = np.r_[
        initial_profile.get("linear_damping_linear", [0, 0, 0]),
        initial_profile.get("linear_damping_angular", [0, 0, 0]),
        initial_profile.get("quadratic_damping_linear", [0, 0, 0]),
        initial_profile.get("quadratic_damping_angular", [0, 0, 0]),
        (initial_profile.get("buoyancy_scale", 1) - 1) * 100,
        initial_profile.get("cob_x_offset", 0) * 100,
        initial_profile.get("cob_z_offset", .02) * 100,
        initial_profile.get("yaw_torque_scale", 1),
        [gains.get(name, 1) for name in names],
        initial_profile.get("cob_y_offset", 0) * 100,
        initial_profile.get("hydro_pitch_moment_coeff", 0),
    ].astype(float)
    count = len(names)
    lower = np.r_[np.zeros(12), -4.8, -2.5, .5, .2, np.full(count, .55), -2, -8]
    upper = np.r_[[100, 140, 180, 5, 5, 8, 400, 500, 600, 35, 35, 40],
                  4.8, 2.5, 8, 1.6, np.full(count, 1.45), 2, 8]
    scale = np.r_[[25, 30, 50, .5, .5, .6, 70, 100, 150, 5, 5, 5],
                  2, 1, 2, .5, np.full(count, .25), 1, 3]
    if not np.isfinite(initial).all() or np.any(initial < lower) or np.any(initial > upper):
        raise ValueError("initial profile contains non-finite values or lies outside fitter bounds")
    if any(np.count_nonzero(np.isfinite(measured), axis=0).min() < 10 for measured, _ in trials):
        raise ValueError("each training channel needs at least ten valid measurements")

    def residual(parameters, long_weight, full_weight):
        values = []
        for (measured, motors), (short, long) in zip(trials, schedules, strict=True):
            thrust = motors * parameters[16:16 + count]
            wrench = thrust @ geometry
            scaled_yaw = wrench[:, 5] if yaw_from_horizontal_only else thrust @ horizontal_yaw_levers
            wrench[:, 5] += scaled_yaw * (parameters[15] - 1.0)
            full = [(short[0][0], len(measured))]
            for weight, schedule in ((1.0, short), (long_weight, long), (full_weight, full)):
                if weight == 0:
                    continue
                for start, stop in schedule:
                    actual = measured[start:stop:5]
                    predicted = _simulate(measured[start], wrench[start:stop], parameters, mass, inertia, com)[::5]
                    errors = (predicted - actual) / np.r_[RATE_SCALE, POSE_SCALE]
                    errors[:, 6:10] *= np.array([.85, .8, .8, .6])
                    errors *= weight / math.sqrt(len(schedule) * len(actual))
                    values.append(errors[np.isfinite(actual)])
        values.append(.14 * (parameters - initial) / scale)
        return np.concatenate(values)

    if max_evaluations > 1:
        warm_budget = max_evaluations // 2
        warm = least_squares(residual, initial, args=(.4, 0), bounds=(lower, upper), x_scale=scale,
                             max_nfev=warm_budget, diff_step=1e-4)
        seed, warm_evaluations = warm.x, warm.nfev
    else:
        seed, warm_evaluations = initial, 0
    final_weights = (.55, .45)
    result = least_squares(residual, seed, args=final_weights, bounds=(lower, upper), x_scale=scale,
                           max_nfev=max_evaluations - warm_evaluations, diff_step=1e-4)
    p = result.x
    profile = copy.deepcopy(initial_profile)
    profile.pop("extends", None)
    profile.update(
        profile_note="Effective submerged model fitted on " + ", ".join(Path(path).name for path in trial_dirs),
        thruster_response_model="rpm", linear_damping_linear=p[:3].tolist(),
        linear_damping_angular=p[3:6].tolist(), quadratic_damping_linear=p[6:9].tolist(),
        quadratic_damping_angular=p[9:12].tolist(), buoyancy_scale=1 + p[12] / 100,
        cob_x_offset=p[13] / 100, cob_z_offset=p[14] / 100, yaw_torque_scale=p[15],
        thruster_direct_gain_scales=dict(zip(names, p[16:16 + count].tolist(), strict=True)),
        cob_y_offset=p[-2] / 100, hydro_pitch_moment_coeff=p[-1],
    )
    parameter_names = [
        "linear_surge", "linear_sway", "linear_heave", "linear_roll", "linear_pitch", "linear_yaw",
        "quadratic_surge", "quadratic_sway", "quadratic_heave", "quadratic_roll", "quadratic_pitch", "quadratic_yaw",
        "buoyancy_percent", "cob_x_cm", "cob_z_cm", "yaw_torque_scale",
        *[f"thrust_gain_{name}" for name in names], "cob_y_cm", "pitch_moment_coefficient",
    ]
    variation = {}
    for index, channel in enumerate((*VELOCITY, *RATE)):
        data = np.concatenate([measured[:, index] for measured, _ in trials])
        data = data[np.isfinite(data)]
        variation[channel] = float(np.percentile(data, 95) - np.percentile(data, 5))
    excitation_floor = [.02, .02, .02, .03, .03, .03]
    weak_axes = [name for name, floor in zip((*VELOCITY, *RATE), excitation_floor, strict=True)
                 if variation[name] < floor]
    singular = np.linalg.svd(result.jac * scale, compute_uv=False)
    rank = int(np.linalg.matrix_rank(result.jac * scale))
    initial_residual = residual(initial, *final_weights)
    seed_residual = residual(seed, *final_weights)
    diagnostics = {
        "method": "bounded least squares on 50 Hz submerged reduced-order dynamics",
        "train_trials": [Path(path).name for path in trial_dirs],
        "mass_kg": mass, "inertia_kg_m2": inertia.tolist(), "center_of_mass_frd_m": com.tolist(),
        "thruster_geometry_frd": geometry.tolist(), "motor_order": names,
        "evaluations": int(warm_evaluations + result.nfev),
        "stages": {"window_warmup_evaluations": int(warm_evaluations), "full_trial_evaluations": int(result.nfev),
                   "final_residual_weights": {"short": 1.0, "long": final_weights[0], "full": final_weights[1]}},
        "success": bool(result.success), "status": int(result.status),
        "message": str(result.message), "initial_cost": float(np.dot(initial_residual, initial_residual) / 2),
        "seed_cost": float(np.dot(seed_residual, seed_residual) / 2),
        "final_cost": float(result.cost),
        "parameters_at_bound": [parameter_names[i] for i in np.flatnonzero(
            np.isclose(p, lower, atol=1e-5) | np.isclose(p, upper, atol=1e-5))],
        "channel_p05_p95_spans": variation, "weakly_excited_channels": weak_axes,
        "scaled_objective_jacobian_rank": rank,
        "scaled_objective_jacobian_singular_values": singular.tolist(),
        "excitation_note": "Small observed motion can leave parameters poorly identified; regularization also contributes to the Jacobian rank.",
        "assumptions": ["fully submerged, no contact or current", "diagonal body-aligned inertia and zero added mass; descendant mass below 0.1% is neglected",
                        "RPM dynamics with fixed 0.04/0.06 second rise/fall priors",
                        "each full-trial rollout is seeded once from the measured initial state",
                        "coarse objective is not the native MuJoCo evaluation"],
    }
    return profile, diagnostics


def refine_yaw_profile(manifest: dict, scene_path: Path, initial_profile: dict,
                       thruster_perf_file: Path, trial_dirs: list[Path],
                       max_evaluations: int) -> tuple[dict, dict]:
    """Refine yaw asymmetry and independently excited horizontal moments.

    Uses only the supplied training runs. The returned profile is a candidate
    that must be checked against the original MuJoCo runtime and held-out data.
    """
    if not trial_dirs or max_evaluations < 1:
        raise ValueError("at least one training trial and a positive evaluation budget are required")
    thrusters = manifest["thrusters"]
    geometry, mass, inertia, com = _geometry(Path(scene_path), thrusters)
    surface = measured_voltage_surface(json.loads(Path(thruster_perf_file).read_text()), include_rpm=True)
    trials = [_read_trial(Path(path), surface, thrusters) for path in trial_dirs]
    schedules = []
    for measured, _ in trials:
        short = _windows(measured, 8, 8)
        long = _windows(measured, 60, 50)
        if not short:
            raise ValueError("every training run needs a complete initial state")
        schedules.append((short, long, [(short[0][0], len(measured))]))

    names = [item["name"] for item in thrusters]
    yaw_names = [item["name"] for item in thrusters if item.get("kind", "horizontal") == "horizontal"]
    if not yaw_names:
        return copy.deepcopy(initial_profile), {
            "method": "bounded training-only yaw refinement", "fitted_parameters": [],
            "train_trials": [Path(path).name for path in trial_dirs],
            "skipped": True, "message": "No horizontal thrusters are available for yaw refinement.",
        }
    gains = initial_profile.get("thruster_direct_gain_scales", {})
    overrides = initial_profile.get("yaw_torque_thruster_scales", {})
    global_yaw_scale = float(initial_profile.get("yaw_torque_scale", 1.0))
    linear_yaw_damping = float(initial_profile.get("linear_damping_angular", [0, 0, 0])[2])
    baseline = np.array([float(overrides.get(name, global_yaw_scale)) for name in yaw_names]
                        + [float(initial_profile.get("yaw_damping_asymmetry", 0.0))])
    lower = np.r_[np.full(len(yaw_names), .05), -.6]
    upper = np.r_[np.full(len(yaw_names), 1.6), .6]
    if not np.isfinite(baseline).all() or np.any(baseline < lower) or np.any(baseline > upper):
        raise ValueError("initial yaw overrides or damping asymmetry lie outside refinement bounds")
    count = len(names)
    parameters = np.r_[
        initial_profile.get("linear_damping_linear", [0, 0, 0]),
        initial_profile.get("linear_damping_angular", [0, 0, 0]),
        initial_profile.get("quadratic_damping_linear", [0, 0, 0]),
        initial_profile.get("quadratic_damping_angular", [0, 0, 0]),
        (initial_profile.get("buoyancy_scale", 1) - 1) * 100,
        initial_profile.get("cob_x_offset", 0) * 100,
        initial_profile.get("cob_z_offset", .02) * 100,
        global_yaw_scale,
        [gains.get(name, 1) for name in names],
        initial_profile.get("cob_y_offset", 0) * 100,
        initial_profile.get("hydro_pitch_moment_coeff", 0),
    ].astype(float)
    motor_gains = parameters[16:16 + count]

    def data_residual(yaw_parameters: np.ndarray) -> np.ndarray:
        yaw_scales = dict(zip(yaw_names, yaw_parameters[:-1], strict=True))
        values = []
        for (measured, motors), schedules_for_trial in zip(trials, schedules, strict=True):
            wrench = _motor_wrench(motors, geometry, motor_gains, thrusters, global_yaw_scale, yaw_scales)
            for weight, schedule in zip((.35, .5, 1.0), schedules_for_trial, strict=True):
                if not schedule:
                    continue
                for start, stop in schedule:
                    actual = measured[start:stop:5]
                    predicted = _simulate(measured[start], wrench[start:stop], parameters, mass, inertia, com,
                                          float(yaw_parameters[-1]))[::5]
                    errors = (predicted[:, (5, 9)] - actual[:, (5, 9)]) / np.array([.32, .9])
                    errors *= weight / math.sqrt(len(schedule) * len(actual))
                    values.append(errors[np.isfinite(actual[:, (5, 9)])])
        return np.concatenate(values)

    yaw_rates = np.concatenate([measured[:, 5] for measured, _ in trials])
    yaw_rates = yaw_rates[np.isfinite(yaw_rates)]
    positive_count = int(np.count_nonzero(yaw_rates > .03))
    negative_count = int(np.count_nonzero(yaw_rates < -.03))
    if max(positive_count, negative_count) < 25:
        return copy.deepcopy(initial_profile), {
            "method": "bounded training-only yaw refinement", "fitted_parameters": [],
            "train_trials": [Path(path).name for path in trial_dirs], "skipped": True,
            "message": "Fewer than 25 samples exceed 0.03 rad/s in either yaw direction.",
            "positive_yaw_rate_samples": positive_count, "negative_yaw_rate_samples": negative_count,
        }
    # Check data sensitivity without regularization. Pivoted QR avoids fitting
    # multiple scales that exert indistinguishable effects on the observed yaw.
    base_residual = data_residual(baseline)
    step = 1e-3
    sensitivity = np.column_stack([
        (data_residual(baseline + np.eye(len(baseline))[index] * step) - base_residual) / step
        for index in range(len(baseline))
    ])
    eligible = list(range(len(yaw_names)))
    if linear_yaw_damping > 1e-8 and min(positive_count, negative_count) >= 25:
        eligible.append(len(yaw_names))
    matrix = sensitivity[:, eligible]
    _, r, pivots = qr(matrix, mode="economic", pivoting=True)
    diagonal = np.abs(np.diag(r))
    cutoff = max(1e-7, (float(diagonal[0]) if len(diagonal) else 0) * 1e-3)
    rank = int(np.count_nonzero(diagonal > cutoff))
    selected = sorted(eligible[int(index)] for index in pivots[:rank])
    parameter_names = [f"yaw_scale_{name}" for name in yaw_names] + ["yaw_damping_asymmetry"]
    if not selected:
        return copy.deepcopy(initial_profile), {
            "method": "bounded training-only yaw refinement", "fitted_parameters": [], "skipped": True,
            "train_trials": [Path(path).name for path in trial_dirs],
            "message": "No yaw parameter has sufficient sensitivity in the supplied training trials.",
            "positive_yaw_rate_samples": positive_count, "negative_yaw_rate_samples": negative_count,
        }
    scale = np.r_[np.full(len(yaw_names), .35), .3]

    def residual(selected_values: np.ndarray) -> np.ndarray:
        candidate = baseline.copy()
        candidate[selected] = selected_values
        return np.r_[data_residual(candidate), .2 * (candidate[selected] - baseline[selected]) / scale[selected]]

    result = least_squares(residual, baseline[selected], bounds=(lower[selected], upper[selected]),
                           x_scale=scale[selected], max_nfev=max_evaluations, diff_step=1e-4)
    refined = baseline.copy()
    refined[selected] = result.x
    profile = copy.deepcopy(initial_profile)
    profile["yaw_torque_thruster_scales"] = dict(zip(yaw_names, refined[:-1].tolist(), strict=True))
    profile["yaw_damping_asymmetry"] = float(refined[-1])
    initial_cost = float(np.dot(base_residual, base_residual) / 2)
    final_residual = data_residual(refined)
    final_cost = float(np.dot(final_residual, final_residual) / 2)
    diagnostics = {
        "method": "bounded training-only yaw refinement on short, long and full 50 Hz trajectories", "skipped": False,
        "train_trials": [Path(path).name for path in trial_dirs],
        "fitted_parameters": [parameter_names[index] for index in selected],
        "fixed_parameters": [parameter_names[index] for index in range(len(baseline)) if index not in selected],
        "positive_yaw_rate_samples": positive_count, "negative_yaw_rate_samples": negative_count,
        "asymmetry_eligible": len(yaw_names) in eligible,
        "sensitivity_rank": rank, "sensitivity_qr_diagonal": diagonal.tolist(), "sensitivity_cutoff": cutoff,
        "initial_data_cost": initial_cost, "final_data_cost": final_cost,
        "evaluations": int(result.nfev), "success": bool(result.success), "message": str(result.message),
        "parameters_at_bound": [parameter_names[index] for index in selected
                                if np.isclose(refined[index], lower[index], atol=1e-5)
                                or np.isclose(refined[index], upper[index], atol=1e-5)],
        "assumptions": ["Fixed non-yaw parameters and global yaw scale", "FRD positive yaw increases linear yaw damping for positive asymmetry",
                        "Sensitivity rank is local to the training data and is not a general identifiability guarantee",
                        "Coarse predictions require native MuJoCo replay and independent validation"],
    }
    return profile, diagnostics
