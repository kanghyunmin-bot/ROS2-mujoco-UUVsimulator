#!/usr/bin/env python3
"""Plot the ArduSub ALT_HOLD contract from a DataFlash BIN log.

This is a read-only analysis tool. It compares the climb rate that the local
ArduSub code should request from RC3 against the logged ALT_HOLD controller
target (CTUN.DCRt), while also showing estimator and servo output signals.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np
from pymavlink import mavutil


RC_MIN = 1100
RC_MAX = 1900
RC_NEUTRAL = 1500
RC3_TRIM = 1100
RC3_DZ = 30
PILOT_SPEED_UP_CM_S = 100.0
PILOT_SPEED_DN_CM_S = 0.0
JS_GAIN_DEFAULT = 0.1
JS_GAIN_MIN = 0.25
JS_GAIN_MAX = 2.0
JS_GAIN_STEPS = 4
MODE_NAMES = {
    2: "ALT_HOLD",
    19: "MANUAL",
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def effective_js_gain() -> float:
    steps = max(1, int(JS_GAIN_STEPS))
    if steps == 1 or (JS_GAIN_DEFAULT < JS_GAIN_MAX + 0.01 and JS_GAIN_DEFAULT > JS_GAIN_MIN - 0.01):
        gain = clamp(JS_GAIN_DEFAULT, JS_GAIN_MIN, JS_GAIN_MAX)
    else:
        gain = JS_GAIN_MIN + (steps / 2.0 - 1.0) * (JS_GAIN_MAX - JS_GAIN_MIN) / float(steps - 1)
    return clamp(gain, 0.1, 1.0)


def pilot_speed_dn() -> float:
    return abs(PILOT_SPEED_DN_CM_S) if PILOT_SPEED_DN_CM_S != 0.0 else abs(PILOT_SPEED_UP_CM_S)


def rc3_to_expected_althold_climb(rc3_pwm: float) -> float:
    if not math.isfinite(rc3_pwm):
        return math.nan
    if rc3_pwm < RC3_TRIM:
        norm = 0.0 if RC_MIN >= RC3_TRIM else (rc3_pwm - RC3_TRIM) / float(RC3_TRIM - RC_MIN)
    else:
        norm = 0.0 if RC_MAX <= RC3_TRIM else (rc3_pwm - RC3_TRIM) / float(RC_MAX - RC3_TRIM)
    norm = clamp(norm, -1.0, 1.0)
    earth_z = 2.0 * (-0.5 + norm)
    throttle_control = 500.0 + PILOT_SPEED_UP_CM_S * earth_z
    center = (RC_MAX + RC_MIN) / 2.0
    target = throttle_control - center + 1000.0
    if abs(target) < RC3_DZ * effective_js_gain():
        target = 0.0
    return clamp(target, -pilot_speed_dn(), PILOT_SPEED_UP_CM_S)


def latest_bin(default_root: Path) -> Path:
    logs = sorted(default_root.glob("*.BIN"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not logs:
        raise SystemExit(f"no BIN logs found in {default_root}")
    return logs[0]


def append_value(streams: dict[str, dict[str, list[float]]], msg_type: str, msg: dict, fields: Iterable[str]) -> None:
    t = float(msg["TimeUS"]) / 1.0e6
    streams[msg_type]["t"].append(t)
    for field in fields:
        streams[msg_type][field].append(float(msg.get(field, math.nan)))


def read_bin(path: Path) -> tuple[dict[str, dict[str, np.ndarray]], list[dict], dict[str, float]]:
    streams: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    modes: list[dict] = []
    params: dict[str, float] = {}
    fields = {
        "RCIN": ["C3"],
        "RCOU": ["C5", "C6", "C7", "C8"],
        "CTUN": ["DAlt", "Alt", "DCRt", "CRt"],
        "PSCD": ["TPD", "PD", "TVD", "VD"],
        "ATT": ["Roll", "Pitch", "DesRoll", "DesPitch"],
        "RATE": ["ROut", "POut", "YOut", "AOut"],
        "VISV": ["VZ", "Ign"],
        "SIM2": ["PD", "VD"],
        "BARO": ["I", "Alt", "Press", "Health"],
    }
    mlog = mavutil.mavlink_connection(str(path))
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        msg_type = msg.get_type()
        data = msg.to_dict()
        if "TimeUS" not in data:
            continue
        if msg_type == "MODE":
            mode_num = int(data.get("ModeNum", data.get("Mode", -1)))
            modes.append(
                {
                    "t": float(data["TimeUS"]) / 1.0e6,
                    "mode": mode_num,
                    "name": MODE_NAMES.get(mode_num, str(mode_num)),
                }
            )
            continue
        if msg_type == "PARM":
            name = str(data.get("Name", ""))
            try:
                params[name] = float(data.get("Value", math.nan))
            except (TypeError, ValueError):
                pass
            continue
        if msg_type in fields:
            append_value(streams, msg_type, data, fields[msg_type])
    if not streams:
        raise SystemExit(f"no usable streams in {path}")
    first_t = min(values["t"][0] for values in streams.values() if values["t"])
    if modes:
        first_t = min(first_t, modes[0]["t"])
    arrays: dict[str, dict[str, np.ndarray]] = {}
    for name, values in streams.items():
        arrays[name] = {}
        for field, vals in values.items():
            arr = np.asarray(vals, dtype=float)
            if field == "t":
                arr = arr - first_t
            arrays[name][field] = arr
    for mode in modes:
        mode["t"] = mode["t"] - first_t
    return arrays, modes, params


def interp(streams: dict[str, dict[str, np.ndarray]], stream: str, field: str, t: np.ndarray) -> np.ndarray:
    if stream not in streams or field not in streams[stream] or len(streams[stream]["t"]) == 0:
        return np.full_like(t, math.nan, dtype=float)
    st = streams[stream]["t"]
    sv = streams[stream][field]
    return np.interp(t, st, sv, left=math.nan, right=math.nan)


def mode_at(times: np.ndarray, modes: list[dict]) -> list[str]:
    if not modes:
        return ["unknown"] * len(times)
    idx = 0
    names = []
    for t in times:
        while idx + 1 < len(modes) and modes[idx + 1]["t"] <= t:
            idx += 1
        names.append(str(modes[idx]["name"]))
    return names


def vertical_plant_cmd_from_rcou(c5: np.ndarray, c6: np.ndarray, c7: np.ndarray, c8: np.ndarray) -> np.ndarray:
    # Down-positive MuJoCo plant command. Negative values correspond to upward thrust.
    signs = np.asarray([-1.0, 1.0, 1.0, -1.0])
    stacked = np.vstack([c5, c6, c7, c8])
    return np.nanmean(((stacked - RC_NEUTRAL) / 400.0) * signs[:, None], axis=0)


def summarize_segments(t: np.ndarray, rc3: np.ndarray, expected: np.ndarray, dcrt: np.ndarray, modes: list[str]) -> list[dict]:
    labels = []
    for mode in sorted(set(modes)):
        if mode == "unknown":
            continue
        for name, mask in [
            ("rc3_high", rc3 > 1550),
            ("rc3_low", rc3 < 1450),
            ("neutral", np.abs(rc3 - RC_NEUTRAL) <= 25),
        ]:
            full_mask = mask & (np.asarray(modes) == mode) & np.isfinite(expected) & np.isfinite(dcrt)
            if not np.any(full_mask):
                continue
            exp_abs = np.nanmean(np.abs(expected[full_mask]))
            dcrt_abs = np.nanmean(np.abs(dcrt[full_mask]))
            labels.append(
                {
                    "mode": mode,
                    "segment": name,
                    "samples": int(np.count_nonzero(full_mask)),
                    "t_start_s": float(np.nanmin(t[full_mask])),
                    "t_end_s": float(np.nanmax(t[full_mask])),
                    "rc3_mean_pwm": float(np.nanmean(rc3[full_mask])),
                    "expected_climb_mean_cm_s": float(np.nanmean(expected[full_mask])),
                    "ctun_dcrt_mean_cm_s": float(np.nanmean(dcrt[full_mask])),
                    "ctun_dcrt_abs_mean_cm_s": float(dcrt_abs),
                    "dcrt_to_expected_abs_ratio": float(dcrt_abs / exp_abs) if exp_abs > 1.0e-6 else math.nan,
                }
            )
    return labels


def write_segment_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def shade_modes(ax, modes: list[dict], end_t: float) -> None:
    if not modes:
        return
    colors = {"ALT_HOLD": "#ffe6e6", "MANUAL": "#e8f2ff"}
    for i, mode in enumerate(modes):
        start = float(mode["t"])
        end = float(modes[i + 1]["t"]) if i + 1 < len(modes) else end_t
        ax.axvspan(start, end, color=colors.get(mode["name"], "#f3f3f3"), alpha=0.35, linewidth=0)
        ax.text(start + 0.4, 0.96, str(mode["name"]), transform=ax.get_xaxis_transform(), fontsize=8, va="top")


def write_plot(
    path: Path,
    t: np.ndarray,
    modes: list[dict],
    rc3: np.ndarray,
    expected: np.ndarray,
    dcrt: np.ndarray,
    pscd_tvd: np.ndarray,
    pscd_vd: np.ndarray,
    sim_pd: np.ndarray,
    sim_vd: np.ndarray,
    visv_vz: np.ndarray,
    rcou: dict[str, np.ndarray],
    plant_cmd: np.ndarray,
    att: dict[str, np.ndarray],
    rate: dict[str, np.ndarray],
) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
    end_t = float(t[-1]) if len(t) else 0.0
    for ax in axes:
        shade_modes(ax, modes, end_t)
        ax.grid(True, alpha=0.25)

    axes[0].plot(t, rc3, color="tab:gray", label="RCIN.C3")
    axes[0].axhline(RC_NEUTRAL, color="black", linewidth=0.8, alpha=0.5)
    axes[0].set_title("Pilot Heave Input")
    axes[0].set_ylabel("PWM")
    axes[0].legend(loc="best", fontsize=8)

    axes[1].plot(t, expected, color="tab:green", label="expected climb from RC3")
    axes[1].plot(t, dcrt, color="tab:red", label="CTUN.DCRt actual target")
    axes[1].plot(t, pscd_tvd, color="tab:purple", alpha=0.75, label="PSCD.TVD")
    axes[1].plot(t, pscd_vd, color="tab:brown", alpha=0.75, label="PSCD.VD")
    axes[1].axhline(0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_title("ALT_HOLD Vertical Target Contract")
    axes[1].set_ylabel("cm/s")
    axes[1].legend(loc="best", fontsize=8)

    axes[2].plot(t, sim_pd, color="tab:red", label="SIM2.PD depth")
    depth_ax = axes[2].twinx()
    depth_ax.plot(t, sim_vd, color="tab:orange", alpha=0.8, label="SIM2.VD")
    depth_ax.plot(t, visv_vz, color="tab:blue", alpha=0.7, label="VISV.VZ")
    axes[2].set_title("Plant Depth and EKF ExternalNav Velocity")
    axes[2].set_ylabel("m down")
    depth_ax.set_ylabel("m/s down")
    lines, labels = axes[2].get_legend_handles_labels()
    lines2, labels2 = depth_ax.get_legend_handles_labels()
    axes[2].legend(lines + lines2, labels + labels2, loc="best", fontsize=8)

    for key, color in zip(["C5", "C6", "C7", "C8"], ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd"]):
        axes[3].plot(t, rcou[key], label=f"RCOU.{key}", color=color)
    axes[3].axhline(RC_NEUTRAL, color="black", linewidth=0.8, alpha=0.5)
    cmd_ax = axes[3].twinx()
    cmd_ax.plot(t, plant_cmd, color="tab:red", label="plant vertical cmd (down +)", linewidth=1.5)
    axes[3].set_title("Vertical Motor Outputs")
    axes[3].set_ylabel("PWM")
    cmd_ax.set_ylabel("norm")
    lines, labels = axes[3].get_legend_handles_labels()
    lines2, labels2 = cmd_ax.get_legend_handles_labels()
    axes[3].legend(lines + lines2, labels + labels2, loc="best", fontsize=8)

    axes[4].plot(t, att["Roll"], label="Roll", color="tab:blue")
    axes[4].plot(t, att["Pitch"], label="Pitch", color="tab:orange")
    rate_ax = axes[4].twinx()
    rate_ax.plot(t, rate["ROut"], label="RATE.ROut", color="tab:green", alpha=0.7)
    rate_ax.plot(t, rate["POut"], label="RATE.POut", color="tab:red", alpha=0.7)
    axes[4].set_title("Attitude Coupling")
    axes[4].set_ylabel("deg")
    rate_ax.set_ylabel("controller output")
    axes[4].set_xlabel("time [s]")
    lines, labels = axes[4].get_legend_handles_labels()
    lines2, labels2 = rate_ax.get_legend_handles_labels()
    axes[4].legend(lines + lines2, labels + labels2, loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", type=Path, default=None, help="DataFlash BIN path. Defaults to latest ardupilot/logs/*.BIN.")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[1] / "logs" / "diagnostics")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(__file__).resolve().parents[3]
    bin_path = args.bin if args.bin is not None else latest_bin(root / "ardupilot" / "logs")
    output_dir = args.output_dir.expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_base = output_dir / f"althold_contract_{stamp}"
    plot_path = out_base.with_suffix(".png")
    summary_path = out_base.with_suffix(".json")
    csv_path = out_base.with_suffix(".csv")

    streams, modes, params = read_bin(bin_path)
    t = streams["RCIN"]["t"]
    rc3 = streams["RCIN"]["C3"]
    expected = np.asarray([rc3_to_expected_althold_climb(v) for v in rc3], dtype=float)
    dcrt = interp(streams, "CTUN", "DCRt", t)
    pscd_tvd = interp(streams, "PSCD", "TVD", t)
    pscd_vd = interp(streams, "PSCD", "VD", t)
    sim_pd = interp(streams, "SIM2", "PD", t)
    sim_vd = interp(streams, "SIM2", "VD", t)
    visv_vz = interp(streams, "VISV", "VZ", t)
    rcou = {key: interp(streams, "RCOU", key, t) for key in ["C5", "C6", "C7", "C8"]}
    plant_cmd = vertical_plant_cmd_from_rcou(rcou["C5"], rcou["C6"], rcou["C7"], rcou["C8"])
    att = {key: interp(streams, "ATT", key, t) for key in ["Roll", "Pitch"]}
    rate = {key: interp(streams, "RATE", key, t) for key in ["ROut", "POut"]}
    mode_names = mode_at(t, modes)
    rows = summarize_segments(t, rc3, expected, dcrt, mode_names)

    write_plot(plot_path, t, modes, rc3, expected, dcrt, pscd_tvd, pscd_vd, sim_pd, sim_vd, visv_vz, rcou, plant_cmd, att, rate)
    write_segment_csv(csv_path, rows)
    summary = {
        "bin": str(bin_path),
        "plot": str(plot_path),
        "segments_csv": str(csv_path),
        "effective_js_gain": effective_js_gain(),
        "assumed_params": {
            "RC3_MIN": RC_MIN,
            "RC3_MAX": RC_MAX,
            "RC3_TRIM": RC3_TRIM,
            "PILOT_SPEED_UP": PILOT_SPEED_UP_CM_S,
            "PILOT_SPEED_DN": PILOT_SPEED_DN_CM_S,
            "JS_GAIN_DEFAULT": JS_GAIN_DEFAULT,
            "JS_GAIN_MIN": JS_GAIN_MIN,
            "JS_GAIN_MAX": JS_GAIN_MAX,
            "JS_GAIN_STEPS": JS_GAIN_STEPS,
        },
        "logged_params_subset": {
            name: params.get(name)
            for name in [
                "AHRS_EKF_TYPE",
                "EK3_SRC1_POSZ",
                "EK3_SRC1_VELZ",
                "VISO_TYPE",
                "SURFACE_DEPTH",
                "PILOT_SPEED_UP",
                "PILOT_SPEED_DN",
                "RC3_MIN",
                "RC3_TRIM",
                "RC3_MAX",
            ]
        },
        "modes": modes,
        "segments": rows,
    }
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"[althold_contract] BIN: {bin_path}")
    print(f"[althold_contract] plot: {plot_path}")
    print(f"[althold_contract] summary: {summary_path}")
    print(f"[althold_contract] segments: {csv_path}")
    for row in rows:
        print(
            "[althold_contract] "
            f"{row['mode']} {row['segment']} rc3={row['rc3_mean_pwm']:.1f} "
            f"expected={row['expected_climb_mean_cm_s']:+.1f}cm/s "
            f"CTUN.DCRt={row['ctun_dcrt_mean_cm_s']:+.1f}cm/s "
            f"abs_ratio={row['dcrt_to_expected_abs_ratio']:.3f}"
        )


if __name__ == "__main__":
    main()
