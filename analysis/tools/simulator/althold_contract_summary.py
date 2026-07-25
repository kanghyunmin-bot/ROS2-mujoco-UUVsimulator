"""Summary payload and console output for ALT_HOLD contract analysis."""

from __future__ import annotations

from pathlib import Path

from althold_contract_model import (
    JS_GAIN_DEFAULT,
    JS_GAIN_MAX,
    JS_GAIN_MIN,
    JS_GAIN_STEPS,
    PILOT_SPEED_DN_CM_S,
    PILOT_SPEED_UP_CM_S,
    RC3_TRIM,
    RC_MAX,
    RC_MIN,
    effective_js_gain,
)


def summary_payload(
    bin_path: Path,
    plot_path: Path,
    csv_path: Path,
    params: dict[str, float],
    modes: list[dict],
    rows: list[dict],
) -> dict:
    return {
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


def print_summary(
    bin_path: Path,
    plot_path: Path,
    summary_path: Path,
    csv_path: Path,
    rows: list[dict],
) -> None:
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


__all__ = ["print_summary", "summary_payload"]
