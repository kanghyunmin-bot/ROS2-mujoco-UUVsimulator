#!/usr/bin/env python3
"""Plot the ArduSub ALT_HOLD contract from a DataFlash BIN log."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from althold_contract_bin import latest_bin, read_bin
from althold_contract_plot import write_plot
from althold_contract_segments import summarize_segments, write_segment_csv
from althold_contract_signals import build_signals
from althold_contract_summary import print_summary, summary_payload


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", type=Path, default=None, help="DataFlash BIN path. Defaults to latest ardupilot/logs/*.BIN.")
    parser.add_argument("--output-dir", type=Path, default=root / "sim" / "current" / "logs" / "diagnostics")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(__file__).resolve().parents[3]
    bin_path = args.bin if args.bin is not None else latest_bin(root / "sim" / "ardupilot" / "logs")
    output_dir = args.output_dir.expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_base = output_dir / f"althold_contract_{stamp}"
    plot_path = out_base.with_suffix(".png")
    summary_path = out_base.with_suffix(".json")
    csv_path = out_base.with_suffix(".csv")

    streams, modes, params = read_bin(bin_path)
    signals = build_signals(streams, modes)
    rows = summarize_segments(
        signals["t"],
        signals["rc3"],
        signals["expected"],
        signals["dcrt"],
        signals["mode_names"],
    )

    write_plot(
        plot_path,
        signals["t"],
        modes,
        signals["rc3"],
        signals["expected"],
        signals["dcrt"],
        signals["pscd_tvd"],
        signals["pscd_vd"],
        signals["sim_pd"],
        signals["sim_vd"],
        signals["visv_vz"],
        signals["rcou"],
        signals["plant_cmd"],
        signals["att"],
        signals["rate"],
    )
    write_segment_csv(csv_path, rows)
    summary = summary_payload(bin_path, plot_path, csv_path, params, modes, rows)
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print_summary(bin_path, plot_path, summary_path, csv_path, rows)


if __name__ == "__main__":
    main()
