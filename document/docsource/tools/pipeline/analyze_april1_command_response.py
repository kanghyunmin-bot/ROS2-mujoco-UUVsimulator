from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_april1_real_bags import read_bag, scalar_stats, vector_stats


DEFAULT_ROOT = Path("real_robot_ros_bag/extracted_2026_04_01")
DEFAULT_OUT = Path("document/docsource/runs/rosbag/real_bag_2026_04_01_command_response")
CONTROL_CHANNELS = (3, 4, 5, 6)
PWM_CENTER = 1500.0
PWM_SPAN = 300.0


def discover_bags(root: Path) -> list[Path]:
    return sorted(root.glob("bag_*/**/*.db3"))


def normalize_rc_override(rc: np.ndarray) -> np.ndarray:
    channels = np.asarray(rc[:, :8], dtype=float)
    return np.clip((channels - PWM_CENTER) / PWM_SPAN, -1.0, 1.0)


def interp_matrix(src_t: np.ndarray, src_v: np.ndarray, dst_t: np.ndarray, lag_s: float) -> tuple[np.ndarray, np.ndarray]:
    query_t = dst_t - float(lag_s)
    mask = (query_t >= src_t[0]) & (query_t <= src_t[-1])
    if np.sum(mask) < 8:
        return mask, np.empty((0, src_v.shape[1]), dtype=float)
    out = np.column_stack([np.interp(query_t[mask], src_t, src_v[:, idx]) for idx in range(src_v.shape[1])])
    return mask, out


def pearson(x: np.ndarray, y: np.ndarray) -> float | None:
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    if x.size < 8 or y.size != x.size or np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def fit_linear(x: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    if x.size < 8 or y.size != x.size or np.std(x) < 1e-12:
        return {"count": int(x.size), "slope": None, "intercept": None, "r2": None}
    A = np.column_stack([np.ones_like(x), x])
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ beta
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return {
        "count": int(x.size),
        "intercept": float(beta[0]),
        "slope": float(beta[1]),
        "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None,
        "residual_std": float(np.std(y - pred)),
    }


def fit_multi(X: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    y = np.asarray(y, dtype=float).reshape(-1)
    X = np.asarray(X, dtype=float)
    if X.shape[0] < 12 or y.size != X.shape[0] or np.std(y) < 1e-12:
        return {"count": int(X.shape[0]), "r2": None}
    A = np.column_stack([np.ones(X.shape[0]), X])
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ beta
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return {
        "count": int(X.shape[0]),
        "intercept": float(beta[0]),
        "coefficients_ch3_ch4_ch5_ch6": [float(v) for v in beta[1:]],
        "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None,
        "residual_std": float(np.std(y - pred)),
    }


def response_series(data: Any) -> dict[str, tuple[np.ndarray, np.ndarray, str]]:
    responses: dict[str, tuple[np.ndarray, np.ndarray, str]] = {}
    t, v = data.array("/dvl/twist:linear_m_s", 3)
    if v.size:
        responses["dvl_x_m_s"] = (t, v[:, 0], "DVL x velocity")
        responses["dvl_y_m_s"] = (t, v[:, 1], "DVL y velocity")
        responses["dvl_z_m_s"] = (t, v[:, 2], "DVL z velocity")
        responses["dvl_speed_m_s"] = (t, np.linalg.norm(v, axis=1), "DVL speed")
    t, v = data.array("/odometry/filtered:linear_m_s", 3)
    if v.size:
        responses["filtered_vx_m_s"] = (t, v[:, 0], "filtered x velocity")
        responses["filtered_vy_m_s"] = (t, v[:, 1], "filtered y velocity")
        responses["filtered_vz_m_s"] = (t, v[:, 2], "filtered z velocity")
    t, v = data.array("/mavros/imu/data:gyro_rad_s", 3)
    if v.size:
        responses["imu_yaw_rate_rad_s"] = (t, v[:, 2], "IMU yaw rate")
        responses["imu_roll_rate_rad_s"] = (t, v[:, 0], "IMU roll rate")
        responses["imu_pitch_rate_rad_s"] = (t, v[:, 1], "IMU pitch rate")
    t, depth = data.array("/depth/pose:depth_positive_m")
    if depth.size >= 5:
        depth = depth.reshape(-1)
        depth_rate = np.gradient(depth) / np.maximum(np.gradient(t), 1e-6)
        responses["depth_m"] = (t, depth, "depth")
        responses["depth_rate_m_s"] = (t, depth_rate, "depth rate")
    return responses


def best_single_channel(rc_t: np.ndarray, cmd8: np.ndarray, target_t: np.ndarray, target_y: np.ndarray) -> list[dict[str, Any]]:
    lags = np.arange(0.0, 2.0001, 0.05)
    results: list[dict[str, Any]] = []
    for channel in range(1, 9):
        if np.std(cmd8[:, channel - 1]) < 1e-12:
            continue
        best: dict[str, Any] | None = None
        for lag in lags:
            mask, X = interp_matrix(rc_t, cmd8[:, [channel - 1]], target_t, float(lag))
            if X.size == 0:
                continue
            y = target_y[mask]
            corr = pearson(X[:, 0], y)
            if corr is None:
                continue
            linear = fit_linear(X[:, 0], y)
            candidate = {
                "channel": channel,
                "lag_s": float(lag),
                "corr": corr,
                "abs_corr": abs(corr),
                **linear,
            }
            if best is None or candidate["abs_corr"] > best["abs_corr"]:
                best = candidate
        if best is not None:
            results.append(best)
    return sorted(results, key=lambda item: item["abs_corr"], reverse=True)


def best_multi_channel(
    rc_t: np.ndarray,
    cmd8: np.ndarray,
    target_t: np.ndarray,
    target_y: np.ndarray,
    channels: tuple[int, ...] = CONTROL_CHANNELS,
) -> dict[str, Any]:
    lags = np.arange(0.0, 2.0001, 0.05)
    cols = [channel - 1 for channel in channels]
    best: dict[str, Any] | None = None
    for lag in lags:
        mask, X = interp_matrix(rc_t, cmd8[:, cols], target_t, float(lag))
        if X.size == 0:
            continue
        y = target_y[mask]
        fitted = fit_multi(X, y)
        if fitted.get("r2") is None:
            continue
        fitted["lag_s"] = float(lag)
        if best is None or float(fitted["r2"]) > float(best["r2"]):
            best = fitted
    return best or {"count": 0, "r2": None}


def isolated_command_stats(
    rc_t: np.ndarray,
    cmd8: np.ndarray,
    response_map: dict[str, tuple[np.ndarray, np.ndarray, str]],
    mapping: dict[int, str],
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    cols = [channel - 1 for channel in CONTROL_CHANNELS]
    for channel, response_name in mapping.items():
        if response_name not in response_map:
            continue
        target_t, target_y, _ = response_map[response_name]
        mask, X = interp_matrix(rc_t, cmd8[:, cols], target_t, 0.4)
        if X.size == 0:
            continue
        channel_col = CONTROL_CHANNELS.index(channel)
        active = np.abs(X[:, channel_col]) >= 0.20
        others = np.ones(X.shape[0], dtype=bool)
        for idx in range(X.shape[1]):
            if idx != channel_col:
                others &= np.abs(X[:, idx]) <= 0.12
        isolated = active & others
        y = target_y[mask][isolated]
        x = X[isolated, channel_col]
        out[f"ch{channel}->{response_name}"] = {
            "sample_count": int(y.size),
            "command_stats": scalar_stats(np.arange(x.size, dtype=float), x),
            "response_stats": scalar_stats(np.arange(y.size, dtype=float), y),
            "linear_fit": fit_linear(x, y),
        }
    return out


def command_summary(rc_t: np.ndarray, rc: np.ndarray, cmd8: np.ndarray) -> dict[str, Any]:
    out: dict[str, Any] = {
        "duration_s": float(rc_t[-1] - rc_t[0]) if rc_t.size >= 2 else 0.0,
        "sample_count": int(rc_t.size),
        "channels": {},
    }
    for channel in range(1, 9):
        pwm = rc[:, channel - 1]
        cmd = cmd8[:, channel - 1]
        out["channels"][f"ch{channel}"] = {
            "pwm": scalar_stats(rc_t, pwm),
            "normalized": scalar_stats(rc_t, cmd),
            "active_fraction_abs_gt_0p1": float(np.mean(np.abs(cmd) > 0.1)),
            "active_fraction_abs_gt_0p2": float(np.mean(np.abs(cmd) > 0.2)),
        }
    return out


def analyze_bag(db_path: Path) -> dict[str, Any]:
    data = read_bag(db_path)
    rc_t, rc = data.array("/mavros/rc/override:channels")
    result: dict[str, Any] = {
        "bag": data.name,
        "db_path": str(db_path),
        "has_rc_override": bool(rc.size),
        "responses": {},
        "inferred_mapping": {},
        "command_summary": {},
        "isolated_axis_checks": {},
    }
    if rc.size == 0:
        return result
    cmd8 = normalize_rc_override(rc)
    result["command_summary"] = command_summary(rc_t, rc, cmd8)
    responses = response_series(data)
    for name, (target_t, target_y, description) in responses.items():
        singles = best_single_channel(rc_t, cmd8, target_t, target_y)
        multi = best_multi_channel(rc_t, cmd8, target_t, target_y)
        result["responses"][name] = {
            "description": description,
            "sample_count": int(target_y.size),
            "target_stats": scalar_stats(target_t, target_y),
            "best_single_channels": singles[:6],
            "multi_ch3_ch4_ch5_ch6": multi,
        }
    result["isolated_axis_checks"] = isolated_command_stats(
        rc_t,
        cmd8,
        responses,
        {
            3: "depth_rate_m_s",
            4: "imu_yaw_rate_rad_s",
            5: "dvl_x_m_s",
            6: "dvl_y_m_s",
        },
    )
    return result


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    try:
        value = float(value)
    except Exception:
        return str(value)
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}g}"


def make_report(results: list[dict[str, Any]], out_dir: Path) -> None:
    lines: list[str] = []
    lines.append("# 2026-04-01 RC Override Command-Response Analysis")
    lines.append("")
    lines.append("Assumption: RC override PWM is normalized as `(pwm - 1500) / 300`, clipped to `[-1, 1]`.")
    lines.append("The main active channels are ch3, ch4, ch5, and ch6.")
    lines.append("")
    lines.append("## Inferred Axis Mapping")
    lines.append("")
    lines.append("| bag | response | best channel | lag s | corr | slope per normalized cmd | R2 single | R2 ch3-6 multi |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|")
    key_responses = [
        "dvl_x_m_s",
        "dvl_y_m_s",
        "dvl_z_m_s",
        "imu_yaw_rate_rad_s",
        "depth_rate_m_s",
        "filtered_vx_m_s",
        "filtered_vy_m_s",
        "filtered_vz_m_s",
    ]
    for result in results:
        for response in key_responses:
            item = result.get("responses", {}).get(response)
            if not item:
                continue
            best = item.get("best_single_channels", [{}])[0]
            multi = item.get("multi_ch3_ch4_ch5_ch6", {})
            lines.append(
                "| {bag} | `{resp}` | {ch} | {lag} | {corr} | {slope} | {r2} | {mr2} |".format(
                    bag=result["bag"],
                    resp=response,
                    ch=best.get("channel", "n/a"),
                    lag=fmt(best.get("lag_s")),
                    corr=fmt(best.get("corr")),
                    slope=fmt(best.get("slope")),
                    r2=fmt(best.get("r2")),
                    mr2=fmt(multi.get("r2")),
                )
            )
    lines.append("")

    lines.append("## Channel Activity")
    lines.append("")
    lines.append("| bag | ch3 active >0.2 | ch4 active >0.2 | ch5 active >0.2 | ch6 active >0.2 |")
    lines.append("|---|---:|---:|---:|---:|")
    for result in results:
        channels = result.get("command_summary", {}).get("channels", {})
        lines.append(
            "| {bag} | {ch3} | {ch4} | {ch5} | {ch6} |".format(
                bag=result["bag"],
                ch3=fmt(channels.get("ch3", {}).get("active_fraction_abs_gt_0p2"), 3),
                ch4=fmt(channels.get("ch4", {}).get("active_fraction_abs_gt_0p2"), 3),
                ch5=fmt(channels.get("ch5", {}).get("active_fraction_abs_gt_0p2"), 3),
                ch6=fmt(channels.get("ch6", {}).get("active_fraction_abs_gt_0p2"), 3),
            )
        )
    lines.append("")

    lines.append("## Concrete Findings")
    lines.append("")
    lines.append("- ch5 is the clearest surge/forward command: it aligns with `/dvl/twist` x velocity with about 0.4-0.45 s lag.")
    lines.append("- ch6 is the clearest sway command: it aligns with `/dvl/twist` y velocity with about 0.35-0.55 s lag.")
    lines.append("- ch4 is the yaw command: it aligns with IMU z angular velocity with about 0.3 s lag and negative sign under the current command convention.")
    lines.append("- ch3/heave is not cleanly identifiable from these bags. In `20-08`, depth-rate correlation is weak to moderate; in `20-20`, ch3 barely varies while forward/yaw/sway dominate. A dedicated vertical step bag is still needed for heave dynamics.")
    lines.append("- Because `/mavros/rc/override` and `/joy` are logged at roughly 100 Hz, these bags are sufficient for replay-style sim-vs-real comparison: feed the same normalized ch3/ch4/ch5/ch6 sequence into the simulator and compare DVL velocity, yaw rate, depth, and filtered odometry after time alignment.")
    lines.append("")

    lines.append("## Isolated Command Checks")
    lines.append("")
    for result in results:
        lines.append(f"### {result['bag']}")
        isolated = result.get("isolated_axis_checks", {})
        if not isolated:
            lines.append("- No isolated command windows found.")
            continue
        for name, item in isolated.items():
            fit = item.get("linear_fit", {})
            lines.append(
                f"- `{name}`: samples `{item.get('sample_count')}`, slope `{fmt(fit.get('slope'))}`, R2 `{fmt(fit.get('r2'))}`, response std `{fmt(item.get('response_stats', {}).get('std'))}`."
            )
        lines.append("")

    (out_dir / "command_response_report.md").write_text("\n".join(lines), encoding="utf-8")


def plot_command_response(db_path: Path, result: dict[str, Any], out_dir: Path) -> str | None:
    data = read_bag(db_path)
    rc_t, rc = data.array("/mavros/rc/override:channels")
    if rc.size == 0:
        return None
    cmd8 = normalize_rc_override(rc)
    dvl_t, dvl = data.array("/dvl/twist:linear_m_s", 3)
    imu_t, gyro = data.array("/mavros/imu/data:gyro_rad_s", 3)
    depth_t, depth = data.array("/depth/pose:depth_positive_m")

    fig, axes = plt.subplots(4, 1, figsize=(13, 9), sharex=True)
    for channel in CONTROL_CHANNELS:
        axes[0].plot(rc_t, cmd8[:, channel - 1], label=f"ch{channel}", lw=0.75)
    axes[0].set_ylabel("norm cmd")
    axes[0].legend(ncol=4, fontsize=8)
    axes[0].grid(True, alpha=0.3)

    if dvl.size:
        axes[1].plot(dvl_t, dvl[:, 0], label="dvl x", lw=0.8)
        axes[1].plot(dvl_t, dvl[:, 1], label="dvl y", lw=0.8)
        axes[1].plot(dvl_t, dvl[:, 2], label="dvl z", lw=0.8)
        axes[1].legend(ncol=3, fontsize=8)
    axes[1].set_ylabel("DVL m/s")
    axes[1].grid(True, alpha=0.3)

    if gyro.size:
        axes[2].plot(imu_t, gyro[:, 2], label="gyro z", lw=0.7)
        axes[2].legend(fontsize=8)
    axes[2].set_ylabel("yaw rad/s")
    axes[2].grid(True, alpha=0.3)

    if depth.size:
        axes[3].plot(depth_t, depth.reshape(-1), label="depth", lw=0.9)
        axes[3].legend(fontsize=8)
    axes[3].set_ylabel("depth m")
    axes[3].set_xlabel("bag time s")
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(result["bag"])
    fig.tight_layout()
    path = out_dir / f"{result['bag']}_command_response.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return str(path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze RC override command-response in April 1 real robot bags.")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    bag_paths = [path for path in discover_bags(args.root) if path.parent.name in {"bag_2026-04-01_20-08-11", "bag_2026-04-01_20-20-30"}]
    results = []
    plot_paths = []
    for db_path in bag_paths:
        print(f"[command-response] {db_path}", flush=True)
        result = analyze_bag(db_path)
        results.append(result)
        if not args.no_plots:
            plot_path = plot_command_response(db_path, result, args.out)
            if plot_path:
                plot_paths.append(plot_path)
    payload = {"results": results, "plots": plot_paths}
    (args.out / "command_response_summary.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    make_report(results, args.out)
    print(f"[command-response] wrote {args.out / 'command_response_report.md'}", flush=True)


if __name__ == "__main__":
    main()
