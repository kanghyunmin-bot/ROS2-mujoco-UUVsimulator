"""Prepare traceable ESC voltage and inspect achieved PWM sampling offline.

Never infer an ESC rail from pack voltage, clip out-of-range observations, or
upsample sparse PWM into purported high-rate evidence. No ROS/FCU connection.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from extract_rosbag_trends import sha256
from sim.physics.thruster_voltage import measured_voltage_surface


def sampling_quality(times, minimum_hz, maximum_gap_s):
    """Inspect actual received packets [s], including outages hidden by average Hz."""
    times = np.asarray(times, dtype=float)
    if len(times) < 2 or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        return {
            "ready": False,
            "samples": len(times),
            "reason": "too_few_or_nonincreasing_timestamps",
        }
    gaps = np.diff(times)
    rate = float((len(times) - 1) / (times[-1] - times[0]))
    return {
        "ready": bool(rate >= minimum_hz and np.max(gaps) <= maximum_gap_s),
        "samples": len(times),
        "received_hz": rate,
        "maximum_gap_s": float(np.max(gaps)),
        "gap_p95_s": float(np.quantile(gaps, 0.95)),
        "gaps_over_limit": int(np.sum(gaps > maximum_gap_s)),
        "minimum_required_hz": minimum_hz,
        "maximum_allowed_gap_s": maximum_gap_s,
    }


def voltage_window(
    times, volts, begin, end, *, reference, provenance, max_gap_s, voltage_range
):
    """Return an interpolated boundary crop only when the whole interval is supported."""
    times, volts = np.asarray(times), np.asarray(volts)
    reasons = []
    if reference not in {"esc_bus", "confirmed_direct_pack"}:
        reasons.append(
            "ESC supply reference is unverified; pack voltage is not converted automatically"
        )
    if not provenance.strip():
        reasons.append("Missing measurement or power-connection provenance")
    if (
        len(times) < 2
        or volts.shape != times.shape
        or not np.all(np.isfinite(times))
        or np.any(np.diff(times) <= 0)
    ):
        return {
            "ready": False,
            "reasons": [*reasons, "Invalid source timestamps or voltage shape"],
        }, None
    if times[0] > begin or times[-1] < end:
        return {
            "ready": False,
            "reasons": [*reasons, "Voltage does not bracket the selected interval"],
        }, None
    left = max(0, int(np.searchsorted(times, begin, side="right")) - 1)
    right = min(len(times) - 1, int(np.searchsorted(times, end, side="left")))
    support_t, support_v = times[left : right + 1], volts[left : right + 1]
    finite = np.isfinite(support_v)
    outside = finite & ((support_v < voltage_range[0]) | (support_v > voltage_range[1]))
    if not np.all(finite):
        reasons.append("Nonfinite voltage inside interpolation support")
    if np.any(outside):
        reasons.append(
            "Voltage exceeds measured force-curve coverage; no clipping or extrapolation"
        )
    largest_gap = float(np.max(np.diff(support_t)))
    if largest_gap > max_gap_s:
        reasons.append("Unsupported voltage gap inside selected interval")
    result = {
        "ready": not reasons,
        "reasons": reasons,
        "support_samples": len(support_t),
        "nonfinite_samples": int(np.sum(~finite)),
        "out_of_curve_samples": int(np.sum(outside)),
        "measured_curve_range_v": list(map(float, voltage_range)),
        "maximum_gap_s": largest_gap,
        "voltage_min_median_max_v": np.quantile(support_v[finite], [0, 0.5, 1]).tolist()
        if np.any(finite)
        else None,
    }
    if reasons:
        return result, None
    crop_t = np.concatenate(([begin], times[(times > begin) & (times < end)], [end]))
    return result, np.column_stack((crop_t - begin, np.interp(crop_t, times, volts)))


def prepare(args):
    if args.output_dir.exists():
        raise ValueError("Refusing to overwrite a prepared replay")
    for name in ("maximum_voltage_gap_s", "minimum_pwm_hz", "maximum_pwm_gap_s"):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            raise ValueError(name + " must be finite and positive")
    source = json.loads((args.source_dir / "source.json").read_text())
    for name, digest in source["derived_sha256"].items():
        if sha256(args.source_dir / name) != digest:
            raise ValueError("Changed extracted source: " + name)
    numeric = np.load(args.source_dir / "numeric.npz", allow_pickle=False)
    key = args.voltage_topic.replace("/", "__")
    battery = numeric[key]
    # Subtract exact integer receipt epochs before converting to seconds.
    time = (numeric[key + "__times_ns"][:, 0] - source["record_origin_ns"]) / 1e9
    rcout_key = "__mavros__rc__out"
    pwm_time = (
        numeric[rcout_key + "__times_ns"][:, 0] - source["record_origin_ns"]
    ) / 1e9
    replay = json.loads(args.replay_json.read_text())
    if not replay.get("complete"):
        raise ValueError("Replay clock reference must be complete")
    begin = float(
        replay["begin_bag_time"] if args.begin_bag_s is None else args.begin_bag_s
    )
    end = float(replay["end_bag_time"] if args.end_bag_s is None else args.end_bag_s)
    if not np.all(np.isfinite([begin, end])) or end <= begin:
        raise ValueError("Expected a finite increasing bag interval")
    time_offset = float(replay["origin_sim_time"] + begin - replay["begin_bag_time"])
    if not np.isfinite(time_offset):
        raise ValueError("Invalid replay clock offset")
    surface = measured_voltage_surface(json.loads(args.performance_json.read_text()))
    status, voltage = voltage_window(
        time,
        battery[:, 1],
        begin,
        end,
        reference=args.voltage_reference,
        provenance=args.provenance,
        max_gap_s=args.maximum_voltage_gap_s,
        voltage_range=surface["voltage_grid"][[0, -1]],
    )
    selected_pwm = pwm_time[(pwm_time >= begin) & (pwm_time <= end)]
    pwm_quality = sampling_quality(
        selected_pwm, args.minimum_pwm_hz, args.maximum_pwm_gap_s
    )
    # Also reject a high-rate burst that covers only a fraction of the interval.
    boundary_covered = bool(
        len(selected_pwm)
        and selected_pwm[0] - begin <= args.maximum_pwm_gap_s
        and end - selected_pwm[-1] <= args.maximum_pwm_gap_s
    )
    pwm_quality["boundary_covered"] = boundary_covered
    pwm_quality["ready"] = pwm_quality["ready"] and boundary_covered
    args.output_dir.mkdir(parents=True)
    with (args.output_dir / "voltage_observations.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["bag_receipt_s", "voltage_v", "current_a"])
        mask = (time >= begin) & (time <= end)
        writer.writerows(zip(time[mask], battery[mask, 1], battery[mask, 2]))
    report = {
        "source_sqlite_sha256": source["sqlite_sha256"],
        "source_manifest_sha256": sha256(args.source_dir / "source.json"),
        "performance_sha256": sha256(args.performance_json),
        "replay_sha256": sha256(args.replay_json),
        "voltage_topic": args.voltage_topic,
        "voltage_reference": args.voltage_reference,
        "provenance": args.provenance,
        "bag_interval_s": [begin, end],
        "voltage_time_offset_s": time_offset,
        "voltage": status,
        "pwm": pwm_quality,
        "voltage_replay_ready": status["ready"],
        "identification_input_ready": bool(status["ready"] and pwm_quality["ready"]),
        "physical_parameters_identified": False,
        "runtime_voltage_csv": None,
    }
    if voltage is not None:
        path = args.output_dir / "esc_voltage.csv"
        np.savetxt(
            path,
            voltage,
            delimiter=",",
            header="sim_time,voltage_v",
            comments="",
            fmt="%.12g",
        )
        metadata = {
            "schema": "uuv_esc_voltage/v1",
            "csv_sha256": sha256(path),
            "voltage_reference": args.voltage_reference,
            "provenance": args.provenance,
            "max_gap_s": args.maximum_voltage_gap_s,
            "outside_trace": args.outside_trace,
            "source_sqlite_sha256": source["sqlite_sha256"],
            "source_topic": args.voltage_topic,
            "bag_interval_s": [begin, end],
            "recommended_time_offset_s": time_offset,
        }
        path.with_suffix(".csv.json").write_text(json.dumps(metadata, indent=2) + "\n")
        report["runtime_voltage_csv"] = str(path)
    (args.output_dir / "preparation.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source_dir", type=Path, required=True)
    parser.add_argument(
        "--replay_json",
        type=Path,
        required=True,
        help="Frozen replay with bag interval and actual simulation origin",
    )
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument(
        "--performance_json",
        type=Path,
        default=ROOT / "config/thruster_performance.json",
    )
    parser.add_argument("--voltage_topic", default="/battery")
    parser.add_argument(
        "--voltage_reference",
        choices=("battery_pack_unverified", "esc_bus", "confirmed_direct_pack"),
        default="battery_pack_unverified",
    )
    parser.add_argument("--provenance", default="")
    parser.add_argument("--begin_bag_s", type=float)
    parser.add_argument("--end_bag_s", type=float)
    parser.add_argument("--maximum_voltage_gap_s", type=float, default=0.2)
    parser.add_argument("--minimum_pwm_hz", type=float, default=50.0)
    parser.add_argument("--maximum_pwm_gap_s", type=float, default=0.05)
    parser.add_argument(
        "--outside_trace",
        choices=("hold", "error"),
        default="hold",
        help="Explicit policy for warmup and after the selected interval",
    )
    result = prepare(parser.parse_args())
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["identification_input_ready"] else 2)
