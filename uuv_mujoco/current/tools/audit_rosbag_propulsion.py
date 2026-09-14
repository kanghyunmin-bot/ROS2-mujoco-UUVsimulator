"""Audit a frozen April-2 propulsion comparison without fitting a new plant.

Reads saved ROS-bag extracts and MuJoCo/SITL traces. The actual-MuJoCo check
is a submerged, fixed-pose actuator probe, not a new closed-loop rollout.
All alternative voltage/polarity/interpolation calculations are sensitivities.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.optimize import lsq_linear

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import check_research_pool_physics as harness
from extract_rosbag_trends import sha256
from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles
from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_SERVO_MAP as NAMES,
)
from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS as SIGNS,
)
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics
from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.runtime.sitl_servo_pwm import packet_commands_from_pwm

DATABASE_SHA = "e3f60d33dbf72f9c4df223f8afe7e389f8c49ffc6ff7031647ce3da32ec3c154"
CURVE_PATH = ROOT / "config/thruster_performance.json"
WINDOW = (39.0, 40.8)  # Short straight segment; not an independent test set.


def summary(values):
    values = np.asarray(values)
    if not values.size or not np.all(np.isfinite(values)):
        raise ValueError("Expected finite, nonempty evidence")
    return {
        "count": len(values),
        "min": np.min(values, axis=0).tolist(),
        "median": np.median(values, axis=0).tolist(),
        "max": np.max(values, axis=0).tolist(),
    }


def select(time, window=WINDOW):
    return (time >= window[0]) & (time <= window[1])


def interpolate(time, values, query):
    if (
        np.any(np.diff(time) <= 0)
        or np.any(query < time[0])
        or np.any(query > time[-1])
    ):
        raise ValueError(
            "Interpolation requires ordered timestamps and covered queries"
        )
    return np.column_stack([np.interp(query, time, col) for col in values.T])


def force_curve(pwm, voltage, polarity="mapped"):
    with contextlib.redirect_stdout(io.StringIO()):
        curve = load_thruster_performance_config(
            CURVE_PATH, requested_voltage=voltage, direct=True
        )
    command = np.clip((np.asarray(pwm) - 1500) / 400, -1, 1) * SIGNS
    if polarity == "mapped":
        lookup = command
        multiplier = 1
    elif polarity in {"all_weaker", "all_stronger"}:
        # Envelope of the two measured hydraulic branches, preserving thrust
        # direction. This does not identify the real prop/wire installation.
        lookup = np.abs(command) * (-1 if polarity == "all_weaker" else 1)
        multiplier = np.sign(command) * (-1 if polarity == "all_weaker" else 1)
    else:
        raise ValueError(polarity)
    force = np.interp(1500 + 400 * lookup, curve["pwm"], curve["force"]) * multiplier
    return np.where(np.abs(command) <= 25 / 400, 0.0, force)


def audit_source_curve(path):
    """Check every supplied 10–20 V knot against original spreadsheet cells."""
    ns = {"s": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    payload = json.loads(CURVE_PATH.read_text())
    checks = []
    with zipfile.ZipFile(path) as archive:
        for index, voltage in enumerate((10, 12, 14, 16, 18, 20), start=2):
            tree = ET.fromstring(archive.read(f"xl/worksheets/sheet{index}.xml"))
            rows = []
            for row in tree.findall("s:sheetData/s:row", ns)[1:]:
                cells = {
                    c.attrib["r"].rstrip("0123456789"): float(c.find("s:v", ns).text)
                    for c in row
                    if c.attrib["r"].rstrip("0123456789") in {"A", "C", "D", "E", "F"}
                }
                rows.append([cells[k] for k in ("A", "C", "D", "E", "F")])
            source = np.asarray(rows)
            stored = next(c for c in payload["curves"] if c["voltage_v"] == voltage)
            np.testing.assert_array_equal(source[:, 0], stored["pwm_us"])
            np.testing.assert_allclose(source[:, 2], voltage, atol=0, rtol=0)
            error = source[:, 4] * 9.80665 - stored["force_n"]
            np.testing.assert_allclose(error, 0, atol=1e-10, rtol=0)
            np.testing.assert_allclose(
                source[:, 1], stored["current_a"], atol=1e-10, rtol=0
            )
            np.testing.assert_allclose(
                source[:, 3], stored["power_w"], atol=1e-10, rtol=0
            )
            checks.append(
                {
                    "voltage_v": voltage,
                    "knots": len(source),
                    "max_force_error_n": float(np.max(np.abs(error))),
                }
            )
    return {
        "source_url": "https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx",
        "source_sha256": sha256(path),
        "kgf_to_newton": 9.80665,
        "checks": checks,
    }


def audit_battery(numeric, origin):
    result = {}
    for key in ("__battery", "__mavros__battery"):
        a = numeric[key]
        t = a[:, 0] - origin
        use = select(t, (20, 70))
        result[key] = {
            "all_count": len(a),
            "drive_voltage_v": summary(a[use, 1]),
            "drive_current_a": summary(a[use, 2]),
            "drive_samples_below_2v": int(np.sum(a[use, 1] < 2)),
            "drive_samples_above_curve_20v": int(np.sum(a[use, 1] > 20)),
            "straight_voltage_v": summary(a[select(t), 1]),
            "straight_current_a": summary(a[select(t), 2]),
        }
    result["interpretation"] = (
        "Preserve topics separately. /mavros/battery contains incompatible low-voltage and pack-voltage values. "
        "/battery is pack telemetry, not a calibrated measurement at each ESC. No clipping or extrapolation "
        "of above-20 V pack samples into the T200 curve is performed."
    )
    return result


def load_trace(case):
    trace = np.genfromtxt(case / "forces.csv", delimiter=",", names=True)
    replay = json.loads((case / "replay.json").read_text())
    if not replay["complete"]:
        raise ValueError("Incomplete rollout")
    time = trace["sim_time"] + replay["begin_bag_time"] - replay["origin_sim_time"]
    if np.any(np.diff(time) <= 0):
        raise ValueError("Unordered trace")
    return time, trace


def channels(trace, names):
    return np.column_stack([trace[n] for n in names])


def fit_sensitivity(fit_dir, rc, origin, gear):
    frozen = np.load(fit_dir / "fit_inputs.npz")
    a, t = frozen["windows"], frozen["t"]
    train = a[:, 0] < 45
    y = a[:, 1]

    def solve(x, mask):
        scale = np.linalg.norm(x[mask], axis=0)
        if np.any(scale < 1e-12):
            raise ValueError("Unexcited fit column")
        fit = lsq_linear(
            x[mask] / scale,
            y[mask],
            bounds=(np.array([0.01, 0]) * scale, np.array([1.5, 1000]) * scale),
        )
        if not fit.success:
            raise ValueError(fit.message)
        return fit.x / scale

    x = a[:, [2, 4]]
    base = solve(x, train)
    expected = json.loads((fit_dir / "fit_result.json").read_text())["models"][
        "thrust_quadratic"
    ]["coefficients"]
    np.testing.assert_allclose(base, expected, rtol=1e-8)
    deletions = []
    for index in np.flatnonzero(train):
        mask = train.copy()
        mask[index] = False
        deletions.append(
            {
                "omitted_start_s": float(a[index, 0]),
                "coefficients_k_d2": solve(x, mask).tolist(),
            }
        )
    variants = []
    for method in ("linear_pwm", "last_observed_pwm"):
        for shift in (-0.25, 0, 0.25):
            query = t - shift
            if method == "linear_pwm":
                pwm = interpolate(rc[:, 0] - origin, rc[:, 1:9], query)
            else:
                indices = np.searchsorted(rc[:, 0] - origin, query, side="right") - 1
                if (
                    np.any(indices < 0)
                    or np.max(query - (rc[indices, 0] - origin)) > 0.6
                ):
                    raise ValueError("Missing PWM support")
                pwm = rc[indices, 1:9]
            force = force_curve(pwm, 20) @ gear[:, 0]
            other = x.copy()
            for i, start in enumerate(a[:, 0]):
                ids = (t >= start - 1e-6) & (t <= start + 1 + 1e-6)
                other[i, 0] = np.trapezoid(force[ids], t[ids])
            coefficient = solve(other, train)
            variants.append(
                {
                    "method": method,
                    "force_delay_s": shift,
                    "coefficients_k_d2": coefficient.tolist(),
                }
            )
    return {
        "frozen_coefficients_k_d2": base.tolist(),
        "train_windows_s": a[train, 0].tolist(),
        "near_neutral_windows_abs_impulse_below_0_2_ns": a[
            train & (np.abs(a[:, 2]) < 0.2), 0
        ].tolist(),
        "leave_one_window_out": deletions,
        "input_sampling_sensitivity": variants,
        "identification_status": "effective_single_bag_only",
        "interpretation": "Same velocity/filter/windows and mass as the original fit. Alternatives are sensitivities, not selected replacements. 2 Hz telemetry cannot establish the true inter-sample PWM or 40–60 ms motor lag.",
    }


def probe_runtime(case, pwm, speed):
    """Check real actuator machinery and the rigid-body equation at fixed pose."""
    import mujoco

    with tempfile.TemporaryDirectory(prefix="uuv-propulsion-probe-") as temporary:
        scene_path = Path(temporary) / "clearance_tank.xml"
        scene_path.write_text(
            (case / "clearance_tank.xml")
            .read_text()
            .replace("/workspace/", str(ROOT.parents[1]) + "/")
        )
        with (
            patch.object(harness, "SCENE", scene_path),
            patch.object(harness, "PROFILE", case / "profiles.json"),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            runtime = harness._build_runtime(
                mujoco,
                profile_name="bag0402_clearance",
                fluid_model="legacy",
                use_custom_hydrodynamics=True,
            )
            runtime.thruster_actuator.perf_cfg = load_thruster_performance_config(
                CURVE_PATH, requested_voltage=20, direct=True
            )
    model, data, actuator = runtime.model, runtime.data, runtime.thruster_actuator
    base, q, v = (
        int(runtime.state.base_id),
        int(runtime.state.world_qpos_adr),
        int(runtime.state.world_qvel_adr),
    )
    data.qpos[q : q + 3] = [0, 0, -0.6]
    data.qpos[q + 3 : q + 7] = [1, 0, 0, 0]
    data.qvel[:] = 0
    data.qvel[v] = speed
    mujoco.mj_forward(model, data)
    actuator.target.update(
        packet_commands_from_pwm(
            all_thruster_names=actuator.all_thruster_names,
            raw_map=NAMES,
            servo_signs=SIGNS,
            pwm_values=pwm,
        )
    )
    actuator.update_forces(10, base_id=base)
    runtime.underwater.apply(harness.DT_S)
    mujoco.mj_forward(model, data)
    if data.ncon:
        raise ValueError("Fixed probe must be contact free")
    expected = force_curve(pwm, 20)
    actual = np.array([actuator.force_cmd[n] for n in NAMES])
    np.testing.assert_allclose(actual, expected, atol=1e-8, rtol=0)
    # Generalized force rotates the force but also translates its moment.
    np.testing.assert_allclose(
        data.qfrc_actuator[v : v + 3], actuator.last_force_body, atol=1e-6, rtol=0
    )
    moment_origin = actuator.last_torque_body + np.cross(
        model.body_ipos[base], actuator.last_force_body
    )
    np.testing.assert_allclose(
        data.qfrc_actuator[v + 3 : v + 6], moment_origin, atol=1e-6, rtol=0
    )
    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, mass, data.qM)
    external = data.qfrc_applied.copy()
    for body in range(1, model.nbody):
        mujoco.mj_applyFT(
            model,
            data,
            data.xfrc_applied[body, :3],
            data.xfrc_applied[body, 3:],
            data.xipos[body],
            body,
            external,
        )
    residual = (
        mass @ data.qacc
        + data.qfrc_bias
        - data.qfrc_passive
        - data.qfrc_actuator
        - external
        - data.qfrc_constraint
    )
    np.testing.assert_allclose(residual, 0, atol=1e-8, rtol=0)
    return {
        "mujoco_version": mujoco.__version__,
        "pwm_us": list(map(int, pwm)),
        "speed_mps": speed,
        "individual_force_n": actual.tolist(),
        "force_body_n": actuator.last_force_body.tolist(),
        "qfrc_actuator_translation_n": data.qfrc_actuator[v : v + 3].tolist(),
        "max_equation_residual_generalized": float(np.max(np.abs(residual))),
        "equation_residual_units": "translational N and rotational N m",
        "max_curve_force_error_n": float(np.max(np.abs(actual - expected))),
        "distributed_surge_drag_n": float(
            runtime.underwater.last_distributed_hydrodynamics_result.force_body_n[0]
        ),
        "added_mass_surge_kg": float(
            runtime.hydrodynamics.full_matrix_hydrodynamics.config.added_mass_6x6[0, 0]
        ),
        "note": "Fixed level pose, zero angular velocity, actual runtime force path. First acceleration-history sample is seeded; no transient or full-trajectory agreement claim.",
    }


def run(args):
    if args.output_dir.exists():
        raise ValueError("Refusing to overwrite audit output")
    manifest = json.loads((args.source_dir / "source.json").read_text())
    if manifest["sqlite_sha256"] != DATABASE_SHA or manifest["decode_errors"] != 0:
        raise ValueError("Expected the frozen April-2 source")
    for name, digest in manifest["derived_sha256"].items():
        if sha256(args.source_dir / name) != digest:
            raise ValueError("Extract changed: " + name)
    origin = manifest["record_origin_ns"] / 1e9
    numeric = np.load(args.source_dir / "numeric.npz")
    rc, dvl = numeric["__mavros__rc__out"], numeric["__dvl__data"]
    time = rc[:, 0] - origin
    use = select(time)
    dt = dvl[:, 0] - origin
    valid = select(dt) & (dvl[:, 5] > 0.5)
    if np.any(select(dt) & (dvl[:, 5] <= 0.5)) or np.max(np.diff(dt[valid])) > 0.3:
        raise ValueError("Straight-segment DVL validity/gap failure")
    speed = float(np.median(dvl[valid, 1]))
    acceleration = float(
        np.polyfit(dt[valid] - np.mean(dt[valid]), dvl[valid, 1], 1)[0]
    )
    nominal_dir = args.replay_dir / "controller"
    profiles, warning = load_sim_profiles(nominal_dir / "profiles.json")
    if warning:
        raise ValueError(warning)
    profile = build_sim_profile(profiles, "bag0402_clearance")
    hyd = DistributedHullHydrodynamics.from_profile(profile)
    scene = ET.parse(nominal_dir / "clearance_tank.xml")
    gears = {
        e.get("name"): np.fromstring(e.get("gear"), sep=" ")[:3]
        for e in scene.findall("./actuator/motor")
    }
    gear = np.array([gears[n] for n in NAMES])
    # The audit's analytical sums use scene-local gears. Reject rotated sites;
    # the separate actual-MuJoCo probe also validates the complete site mapping.
    for motor in scene.findall("./actuator/motor"):
        if motor.get("name") not in NAMES:
            continue
        site = scene.find(f".//site[@name='{motor.get('site')}']")
        if site is None or any(
            k in site.attrib for k in ("euler", "axisangle", "xyaxes", "zaxis")
        ):
            raise ValueError("Analytical gear audit requires unrotated thruster sites")
        np.testing.assert_allclose(
            np.fromstring(site.get("quat", "1 0 0 0"), sep=" "), [1, 0, 0, 0], atol=0
        )
    np.testing.assert_allclose(np.linalg.norm(gear, axis=1), 1, atol=1e-8)
    np.testing.assert_allclose(gear[4:, 0], 0, atol=0)
    drag = hyd.evaluate(
        body_position_world_m=np.array([0, 0, -0.6]),
        rotation_world_from_body=np.eye(3),
        linear_velocity_world_mps=np.array([speed, 0, 0]),
        angular_velocity_world_radps=np.zeros(3),
        current_world_mps=np.zeros(3),
        surface_height_world_m=0,
        time_s=0,
    )
    drag_n = -float(drag.force_body_n[0])
    mass = (
        sum(c["mass"] for c in profile["body_components"])
        + profile["hydrodynamic_matrices"]["added_mass_6x6"][0][0]
    )
    sensitivity = []
    for voltage in (10, 12, 16, 20):
        for polarity in ("mapped", "all_weaker", "all_stronger"):
            surge = force_curve(rc[use, 1:9], voltage, polarity) @ gear[:, 0]
            sensitivity.append(
                {
                    "voltage_v": voltage,
                    "polarity": polarity,
                    "mean_surge_force_n": float(np.mean(surge)),
                    "conditional_unexplained_opposing_force_n": float(
                        np.mean(surge) - drag_n - mass * acceleration
                    ),
                }
            )
    cases, traces = {}, {}
    for label, case in (
        ("nominal", nominal_dir),
        ("effective", args.replay_dir / "accepted"),
    ):
        ct, trace = load_trace(case)
        traces[label] = (ct, trace)
        pwm = channels(trace, [f"sitl_ch{i}_pwm" for i in range(1, 9)])
        sim_pwm = interpolate(ct, pwm, time[use])
        error = sim_pwm[:, :4] - rc[use, 1:5]
        sampled_speed = interpolate(ct, trace["lin_vel_body_x"][:, None], dt[valid])[
            :, 0
        ]
        # Reconstruct static force from saved drive, separating it from PWM lag.
        state = channels(trace, [n + "_state" for n in NAMES])
        gain = channels(trace, [n + "_direct_gain" for n in NAMES])
        expected_force = force_curve(1500 + state * 400 / SIGNS, 20) * gain
        immersion = channels(trace, [n + "_immersion_scale" for n in NAMES])
        inflow = channels(trace, [n + "_inflow_multiplier" for n in NAMES])
        recorded_force = channels(trace, [n + "_force" for n in NAMES])
        active = select(ct, (20, 70))
        reconstruction = np.abs(
            (expected_force * immersion * inflow - recorded_force)[active]
        )
        force_error = np.abs(
            (
                channels(
                    trace,
                    ["thr_force_world_x", "thr_force_world_y", "thr_force_world_z"],
                )
                - channels(trace, [f"qfrc_actuator_{i}" for i in range(3)])
            )[active]
        )
        if np.max(reconstruction) > 1e-5 or np.max(force_error) > 1e-4:
            raise ValueError(
                "Saved actuator trace no longer satisfies the force contract"
            )
        all_drive = select(time, (20, 70))
        drive_error = interpolate(ct, pwm, time[all_drive]) - rc[all_drive, 1:9]
        cases[label] = {
            "drive_pwm_samples": int(np.sum(all_drive)),
            "drive_pwm_rmse_per_channel_us": np.sqrt(
                np.mean(drive_error**2, axis=0)
            ).tolist(),
            "straight_horizontal_pwm_rmse_us": float(np.sqrt(np.mean(error**2))),
            "straight_horizontal_pwm_max_error_us": float(np.max(np.abs(error))),
            "straight_surge_speed_mps": summary(sampled_speed),
            "straight_surge_rmse_mps": float(
                np.sqrt(np.mean((sampled_speed - dvl[valid, 1]) ** 2))
            ),
            "straight_surge_force_n": summary(trace["thr_force_body_x"][select(ct)]),
            "max_saved_drive_to_force_error_n": float(np.max(reconstruction)),
            "max_saved_force_to_mujoco_translation_error_n": float(np.max(force_error)),
            "trace_sha256": sha256(case / "forces.csv"),
            "replay_sha256": sha256(case / "replay.json"),
        }
    median_pwm = np.rint(np.median(rc[use, 1:9], axis=0)).astype(int)
    probe = probe_runtime(nominal_dir, median_pwm, speed)
    expected_force = force_curve(median_pwm, 20) @ gear
    np.testing.assert_allclose(probe["force_body_n"], expected_force, atol=1e-6, rtol=0)
    report = {
        "status": "propulsion_budget_mismatch_localized_not_physically_identified",
        "source_sqlite_sha256": DATABASE_SHA,
        "source_manifest_sha256": sha256(args.source_dir / "source.json"),
        "audit_script_sha256": sha256(Path(__file__)),
        "curve_audit": audit_source_curve(args.performance_xlsx),
        "battery": audit_battery(numeric, origin),
        "straight_segment": {
            "window_s": list(WINDOW),
            "pwm_samples": int(np.sum(use)),
            "real_surge_mps": summary(dvl[valid, 1]),
            "real_horizontal_pwm_us": summary(rc[use, 1:5]),
            "real_velocity_linear_slope_mps2": acceleration,
            "conditional_mass_kg": mass,
            "nominal_drag_at_real_speed_n": drag_n,
            "voltage_polarity_sensitivity": sensitivity,
        },
        "saved_rollouts": cases,
        "actual_mujoco_probe": probe,
        "fit_sensitivity": fit_sensitivity(args.replay_dir, rc, origin, gear),
        "configuration_sha256": {
            str(p.relative_to(ROOT)): sha256(p)
            for p in (
                CURVE_PATH,
                ROOT / "config/thruster_params.json",
                ROOT / "config/sim_profiles.json",
            )
        },
        "case_input_sha256": {
            p.name: sha256(p)
            for p in (
                nominal_dir / "profiles.json",
                nominal_dir / "clearance_tank.xml",
                nominal_dir / "params.parm",
                args.replay_dir / "fit_inputs.npz",
                args.replay_dir / "fit_result.json",
            )
        },
        "limits": [
            "No new plant coefficients applied; frozen fit is retained as an effective model only.",
            "Short straight segment is diagnostic, not proof that all RC/PWM matches, and not a separate real trial.",
            "Residual opposing force is conditional on nominal static thrust, zero water current, mass and straight-body approximation; it is not measured tether tension.",
            "Real vehicle had a tether; diagnostic scene has none. Mounted prop polarity, ESC bus voltage, net thrust, hull drag and water current remain unmeasured.",
            "Full pose/trajectory Real2Sim, depth/yaw and sim-to-real policy success remain unvalidated.",
        ],
    }
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "audit.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    plot(report, rc, dvl, origin, traces, args.output_dir / "propulsion.png")
    print(
        json.dumps(
            {
                k: report[k]
                for k in (
                    "straight_segment",
                    "saved_rollouts",
                    "actual_mujoco_probe",
                    "fit_sensitivity",
                )
            },
            indent=2,
        )
    )


def plot(report, rc, dvl, origin, traces, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {"font.size": 10, "axes.spines.top": False, "axes.spines.right": False}
    )
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained")
    fig.suptitle(
        "April 2 Real-to-Sim propulsion audit | frozen model, no retuning", fontsize=16
    )
    ax = axes[0, 0]
    rt = rc[:, 0] - origin
    use = select(rt, (38, 42))
    ax.plot(
        rt[use],
        np.mean(rc[use, 1:5], axis=1),
        "o-",
        color="#d96538",
        label="Real RCOut (2 Hz)",
    )
    for label, color in (("nominal", "#737d89"), ("effective", "#008f91")):
        t, row = traces[label]
        mask = select(t, (38, 42))
        pwm = channels(row, [f"sitl_ch{i}_pwm" for i in range(1, 5)])
        ax.plot(t[mask], np.mean(pwm[mask], axis=1), color=color, label=label.title())
    ax.set(
        title="Horizontal motor commands agree in the straight window",
        xlabel="Bag receipt time [s]",
        ylabel="Mean horizontal PWM [us]",
    )
    ax.axvspan(*WINDOW, color="#b8c8df", alpha=0.2)
    ax.legend(fontsize=9)
    ax = axes[0, 1]
    dt = dvl[:, 0] - origin
    valid = select(dt, (20, 41)) & (dvl[:, 5] > 0.5)
    ax.plot(dt[valid], dvl[valid, 1], ".", color="#d96538", label="Real DVL")
    for label, color in (("nominal", "#737d89"), ("effective", "#008f91")):
        t, row = traces[label]
        mask = select(t, (20, 41))
        ax.plot(t[mask], row["lin_vel_body_x"][mask], color=color, label=label.title())
    ax.axvspan(*WINDOW, color="#b8c8df", alpha=0.2)
    ax.set(
        title="Same command does not produce the same speed",
        xlabel="Bag receipt time [s]",
        ylabel="Surge speed [m/s]",
    )
    ax.legend(fontsize=9)
    ax = axes[1, 0]
    segment = report["straight_segment"]
    variants = segment["voltage_polarity_sensitivity"]
    mapped = [v for v in variants if v["polarity"] == "mapped"]
    force = [v["mean_surge_force_n"] for v in mapped]
    ax.bar([str(v["voltage_v"]) + " V" for v in mapped], force, color="#78879c")
    required = (
        segment["nominal_drag_at_real_speed_n"]
        + segment["conditional_mass_kg"] * segment["real_velocity_linear_slope_mps2"]
    )
    ax.axhline(
        required,
        color="#d96538",
        ls="--",
        label="Nominal drag + inertia at real motion",
    )
    ax.set(
        title="Measured voltage curves: conditional thrust budget",
        ylabel="Force [N]",
        xlabel="Assumed ESC voltage; not measured ESC-bus values",
    )
    for i, f in enumerate(force):
        ax.text(i, f + 0.3, f"{f:.1f}", ha="center")
    ax.legend(fontsize=8)
    ax = axes[1, 1]
    loo = report["fit_sensitivity"]["leave_one_window_out"]
    ax.bar(
        [str(int(v["omitted_start_s"])) for v in loo],
        [v["coefficients_k_d2"][0] for v in loo],
        color="#008f91",
    )
    ax.axhline(
        report["fit_sensitivity"]["frozen_coefficients_k_d2"][0],
        color="#364152",
        ls="--",
        label="Frozen k = 0.123",
    )
    ax.set(
        title="0.123 depends on very few excited windows",
        xlabel="Omitted 1-second training window start [s]",
        ylabel="Refitted effective thrust multiplier",
    )
    ax.legend(fontsize=9)
    fig.savefig(path, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source_dir", type=Path, required=True)
    parser.add_argument("--replay_dir", type=Path, required=True)
    parser.add_argument("--performance_xlsx", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    run(parser.parse_args())
