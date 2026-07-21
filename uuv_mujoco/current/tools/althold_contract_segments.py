"""Segment summaries for ALT_HOLD contract analysis."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

from althold_contract_model import RC_NEUTRAL


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


__all__ = ["summarize_segments", "write_segment_csv"]
