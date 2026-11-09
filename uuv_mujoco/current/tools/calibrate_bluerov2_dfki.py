"""Fit an effective submerged BlueROV2 model on selected DFKI PWM trials.

The coarse integration is a search model. Score the exported profile using
replay_bluerov2_dfki.py to measure the actual MuJoCo runtime response.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.optimize import least_squares

from replay_bluerov2_dfki import ANGLE, AXES, RATE, RUNTIME, VELOCITY, read_rows
from rosbag_trend_math import interpolate_valid

sys.path.insert(0, str(RUNTIME))
from physics.thruster_mapping import BLUEROV2_DFKI_SERVO_SIGNS, BLUEROV2_SERVO_MAP
from sim.physics.thruster_dynamics import thruster_response
from sim.physics.thruster_voltage import measured_voltage_surface, select_supply_voltage

try:
    from numba import njit
except ImportError:
    def njit(function):
        return function

DT = .02
MASS = 10.012
INERTIA = np.array([.099, .178, .269])
WEIGHT = MASS * 9.81
# Linear damping, quadratic damping, fractional buoyancy offset [percent],
# CoB x/z [cm], yaw lever scale, six effective installed thrust gains,
# CoB y [cm], and pitch moment coefficient.
PRIOR = np.array([1.3125, 9.1435, 2.015, .08, .10, .12,
                  38.169, 129.6607, 243.25, 3.992, 3.992, 4.86,
                  0, 0, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0])
LOWER = np.array([0] * 12 + [-4.8, -2.5, .5, .2] + [.55] * 6 + [-2, -8])
UPPER = np.array([100, 140, 180, 5, 5, 8, 400, 500, 600, 35, 35, 40,
                  4.8, 2.5, 8, 1.6] + [1.45] * 6 + [2, 8])
SCALE = np.array([25, 30, 50, .5, .5, .6, 70, 100, 150, 5, 5, 5,
                  2, 1, 2, .5] + [.25] * 6 + [1, 3])
RATE_SCALE = np.array([.18, .09, .10, .13, .11, .32])
POSE_SCALE = np.array([.7, .11, .12, .9])


def read_trial(path: Path, surface: dict) -> tuple[np.ndarray, np.ndarray]:
    real, pwm = (read_rows(path / f"{name}.csv") for name in ("real", "pwm"))
    voltage = np.genfromtxt(path / "voltage.csv", delimiter=",", names=True, ndmin=1)
    start = real["t_s"][0]
    time = np.arange(start, real["t_s"][-1], DT)
    measured = np.column_stack([
        interpolate_valid(real["t_s"], real[field], time, max_gap_s=.04)
        for field in (*VELOCITY, *RATE, "depth_m", *ANGLE)
    ])
    measured[:, 7:10] = np.deg2rad(measured[:, 7:10])
    packet = np.searchsorted(pwm["t_s"], time + 1e-7, side="right") - 1
    pwms = np.column_stack([pwm[f"pwm{i}"][packet.clip(min=0)] for i in range(1, 7)])
    pwms[(packet < 0)] = 1500
    volts = np.interp(time - start, voltage["sim_time"], voltage["voltage_v"])
    rpm = np.zeros(6)
    force = np.zeros_like(pwms)
    curve_pulses = 1500 + (pwms - 1500) * BLUEROV2_DFKI_SERVO_SIGNS
    for i, (row, v) in enumerate(zip(curve_pulses, volts, strict=True)):
        select_supply_voltage(surface, float(v))
        target = np.interp(row, surface["pwm"], surface["rpm"])
        target[np.abs(row - 1500) <= 25] = 0.0
        rpm = np.array([thruster_response(a, b, DT, .04, .06) for a, b in zip(rpm, target, strict=True)])
        force[i] = np.interp(rpm, surface["rotor_rpm_knots"], surface["rotor_force_knots"])
    return measured, force


def thrust_geometry() -> np.ndarray:
    path = RUNTIME / "scenes" / "bluerov2_dfki_scene.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    center = model.body("base_link").ipos
    rows = []
    for name in BLUEROV2_SERVO_MAP:
        direction = model.actuator(name).gear[:3]
        lever = model.site(f"thr_{name}").pos - center
        rows.append(np.r_[direction * AXES, np.cross(lever, direction) * AXES])
    return np.asarray(rows)


@njit
def simulate(initial, thrust, parameters, implicit_damping):
    """Integrate center-of-mass dynamics in FRD, reporting the base-origin twist."""
    x = initial.copy()
    out = np.empty((len(thrust), 10))
    dl, dq = parameters[:6], parameters[6:12]
    buoyancy = WEIGHT * (1.0 + parameters[12] / 100)
    cob_x, cob_z = parameters[13] / 100, -parameters[14] / 100
    cob_y = -parameters[22] / 100
    com_z = -.011
    x[0] += x[4] * com_z
    x[1] -= x[3] * com_z
    start_depth = initial[6]
    for i in range(len(thrust)):
        u, v, w, p, q, r, depth, roll, pitch, yaw = x
        out[i] = x
        out[i, 0] -= q * com_z
        out[i, 1] += p * com_z
        sr, cr, st, ct = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
        ct = max(.06, ct)
        down_x, down_y, down_z = -st, sr * ct, cr * ct
        net = WEIGHT - buoyancy
        damping = dl + dq * np.abs(x[:6])
        wrench = thrust[i].copy() if implicit_damping else thrust[i] - damping * x[:6]
        wrench[5] += thrust[i, 5] * (parameters[15] - 1.0)
        x[0] += DT * ((wrench[0] + net * down_x) / MASS - q * w + r * v)
        x[1] += DT * ((wrench[1] + net * down_y) / MASS - r * u + p * w)
        x[2] += DT * ((wrench[2] + net * down_z) / MASS - p * v + q * u)
        x[3] += DT * ((wrench[3] + buoyancy * (cob_z * down_y - cob_y * down_z)
                      + (INERTIA[1] - INERTIA[2]) * q * r) / INERTIA[0])
        x[4] += DT * ((wrench[4] - buoyancy * (cob_z * down_x - cob_x * down_z)
                      - parameters[23] * u * abs(u) + (INERTIA[2] - INERTIA[0]) * p * r) / INERTIA[1])
        x[5] += DT * ((wrench[5] - buoyancy * (cob_x * down_y - cob_y * down_x)
                      + (INERTIA[0] - INERTIA[1]) * p * q) / INERTIA[2])
        if implicit_damping:
            x[:3] /= 1 + DT * damping[:3] / MASS
            x[3:6] /= 1 + DT * damping[3:6] / INERTIA
        x[6] += DT * (u * down_x + v * down_y + w * down_z)
        out[i, 6] = depth - com_z * (down_z - math.cos(initial[7]) * math.cos(initial[8]))
        yaw_rate = (sr * q + cr * r) / ct
        x[7] += DT * (p + st * yaw_rate)
        x[8] += DT * (cr * q - sr * r)
        x[9] += DT * yaw_rate
        if np.max(np.abs(x[:6])) > 6 or abs(x[8]) > 1.50 or abs(x[6] - start_depth) > 35:
            out[i + 1:] = out[i]
            break
    return out


def windows(measured: np.ndarray, long: bool = False) -> list[tuple[int, int]]:
    duration = 60 if long else 8
    stride = 50 if long else 8
    length = round(duration / DT)
    result = []
    for i in range(0, len(measured) - round(3 / DT), round(stride / DT)):
        seed = np.flatnonzero(np.isfinite(measured[i:i + 50]).all(axis=1))
        if seed.size:
            start = i + int(seed[0])
            stop = min(start + length, len(measured))
            result.append((start, stop))
    return result


def fit(trials, geometry, initial, long_weight, full_weight, max_evaluations, implicit_damping):
    schedules = [(windows(measured), windows(measured, long=True)) for measured, _ in trials]

    def residual(parameters):
        values = []
        for (measured, motors), (short, long) in zip(trials, schedules, strict=True):
            wrench = (motors * parameters[16:22]) @ geometry
            full = [(short[0][0], len(measured))]
            for weight, schedule in ((1.0, short), (long_weight, long), (full_weight, full)):
                if weight == 0:
                    continue
                for start, stop in schedule:
                    actual = measured[start:stop:5]
                    predicted = simulate(measured[start], wrench[start:stop], parameters, implicit_damping)[::5]
                    scale = np.r_[RATE_SCALE, POSE_SCALE]
                    errors = (predicted - actual) / scale
                    errors[:, 6:10] *= np.array([.85, .8, .8, .6])
                    errors *= weight / math.sqrt(len(schedule) * len(actual))
                    values.append(errors[np.isfinite(actual)])
        values.append(.14 * (parameters - PRIOR) / SCALE)
        return np.concatenate(values)

    fit_result = least_squares(residual, initial, bounds=(LOWER, UPPER), x_scale=SCALE,
                               max_nfev=max_evaluations, diff_step=1e-4, verbose=1)
    return fit_result.x


def export_profile(parameters: np.ndarray, trials: list[str]) -> dict:
    path = RUNTIME / "config" / "sim_profiles_bluerov2.json"
    profile = json.loads(path.read_text())["bluerov2_standard"]
    profile.update(
        profile_note=f"Effective submerged DFKI plant fit on {', '.join(trials)}. No tether model.",
        thruster_response_model="rpm",
        linear_damping_linear=parameters[:3].tolist(),
        linear_damping_angular=parameters[3:6].tolist(),
        quadratic_damping_linear=parameters[6:9].tolist(),
        quadratic_damping_angular=parameters[9:12].tolist(),
        buoyancy_scale=1.0 + parameters[12] / 100,
        cob_x_offset=parameters[13] / 100,
        cob_y_offset=parameters[22] / 100,
        cob_z_offset=parameters[14] / 100,
        yaw_torque_scale=parameters[15],
        hydro_pitch_moment_coeff=parameters[23],
        thruster_direct_gain_scales=dict(zip(BLUEROV2_SERVO_MAP, parameters[16:22].tolist(), strict=True)),
    )
    return {"bluerov2_standard": profile}


def seed_parameters(path: Path | None) -> np.ndarray:
    if path is None:
        return PRIOR.copy()
    profile = json.loads(path.read_text())["bluerov2_standard"]
    gains = profile.get("thruster_direct_gain_scales", {})
    return np.clip(np.r_[
        profile["linear_damping_linear"], profile["linear_damping_angular"],
        profile["quadratic_damping_linear"], profile["quadratic_damping_angular"],
        (profile["buoyancy_scale"] - 1) * 100,
        profile["cob_x_offset"] * 100, profile["cob_z_offset"] * 100,
        profile["yaw_torque_scale"], [gains.get(name, 1) for name in BLUEROV2_SERVO_MAP],
        profile.get("cob_y_offset", 0) * 100, profile.get("hydro_pitch_moment_coeff", 0),
    ], LOWER, UPPER)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--train", nargs="+", default=["random", "circle", "10m"])
    parser.add_argument("--max-evaluations", type=int, default=70)
    parser.add_argument("--long-weight", type=float, default=.4)
    parser.add_argument("--full-weight", type=float, default=0.0)
    parser.add_argument("--seed-profile", type=Path)
    parser.add_argument("--implicit-damping", action="store_true")
    args = parser.parse_args()
    perf = json.loads((RUNTIME / "config" / "thruster_performance.json").read_text())
    surface = measured_voltage_surface(perf, include_rpm=True)
    trials = [read_trial(args.data / trial, surface) for trial in args.train]
    geometry = thrust_geometry()
    parameters = fit(
        trials, geometry, seed_parameters(args.seed_profile), args.long_weight, args.full_weight,
        args.max_evaluations, args.implicit_damping,
    )
    result = export_profile(parameters, args.train)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
