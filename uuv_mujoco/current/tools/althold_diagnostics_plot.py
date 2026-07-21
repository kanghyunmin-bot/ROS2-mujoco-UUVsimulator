"""Plot output for ALT_HOLD diagnostics."""

from __future__ import annotations

import sys
from pathlib import Path

from althold_diagnostics_contract import RC_NEUTRAL, Snapshot


def write_plot(path: Path, rows: list[Snapshot]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        print("[althold_diag] matplotlib not available; skipped plot", file=sys.stderr)
        return
    if not rows:
        return
    t = [r.t_s for r in rows]
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True)

    _plot_pilot_input(axes[0], t, rows)
    _plot_depth(axes[1], t, rows)
    _plot_vertical_velocity(axes[2], t, rows)
    _plot_attitude(axes[3], t, rows)
    _plot_vertical_servo(axes[4], t, rows)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _plot_pilot_input(axis, t: list[float], rows: list[Snapshot]) -> None:
    axis.plot(t, [r.manual_z for r in rows], label="MANUAL_CONTROL.z", color="tab:blue")
    axis.step(t, [r.manual_expected_rc3 for r in rows], where="post", label="expected RC3 from manual", color="tab:green")
    axis.step(t, [r.rc_in_ch3 for r in rows], where="post", label="/mavros/rc/in ch3", color="tab:gray")
    axis.set_title("Pilot input")
    axis.set_ylabel("axis / PWM")
    axis.legend(loc="best", fontsize=8)


def _plot_depth(axis, t: list[float], rows: list[Snapshot]) -> None:
    axis.plot(t, [r.depth_bar30_m for r in rows], label="/depth Bar30", color="tab:blue")
    axis.plot(t, [r.mujoco_depth_m for r in rows], label="MuJoCo depth", color="tab:red")
    axis.plot(t, [r.sim_odom_depth_m for r in rows], label="/sim/odom depth", color="tab:orange")
    axis.set_title("Depth")
    axis.set_ylabel("m down")
    axis.legend(loc="best", fontsize=8)


def _plot_vertical_velocity(axis, t: list[float], rows: list[Snapshot]) -> None:
    axis.plot(t, [r.mavros_velz_down_mps for r in rows], label="local velocity z down", color="tab:purple")
    axis.plot(t, [r.dvl_velz_down_mps for r in rows], label="DVL z down", color="tab:brown")
    axis.plot(t, [r.mujoco_depth_rate_down_mps for r in rows], label="MuJoCo dz/dt down", color="tab:red")
    axis.set_title("Vertical velocity")
    axis.set_ylabel("m/s down")
    axis.legend(loc="best", fontsize=8)


def _plot_attitude(axis, t: list[float], rows: list[Snapshot]) -> None:
    rad_to_deg = 180.0 / 3.141592653589793
    axis.plot(t, [r.mujoco_roll_rad * rad_to_deg for r in rows], label="MuJoCo roll", color="tab:blue")
    axis.plot(t, [r.mujoco_pitch_rad * rad_to_deg for r in rows], label="MuJoCo pitch", color="tab:cyan")
    axis.plot(t, [r.sitl_att_roll_rad * rad_to_deg for r in rows], label="SITL ATT roll", color="tab:red")
    axis.plot(t, [r.sitl_att_pitch_rad * rad_to_deg for r in rows], label="SITL ATT pitch", color="tab:orange")
    rate_axis = axis.twinx()
    rate_axis.plot(
        t,
        [r.sitl_att_rollspeed_rad_s for r in rows],
        label="SITL rollspeed",
        color="tab:purple",
        alpha=0.55,
        linewidth=1.0,
    )
    rate_axis.plot(
        t,
        [r.sitl_att_pitchspeed_rad_s for r in rows],
        label="SITL pitchspeed",
        color="tab:brown",
        alpha=0.55,
        linewidth=1.0,
    )
    axis.set_title("Attitude contract")
    axis.set_ylabel("deg")
    rate_axis.set_ylabel("rad/s")
    attitude_lines, attitude_labels = axis.get_legend_handles_labels()
    rate_lines, rate_labels = rate_axis.get_legend_handles_labels()
    axis.legend(attitude_lines + rate_lines, attitude_labels + rate_labels, loc="best", fontsize=8)


def _plot_vertical_servo(axis, t: list[float], rows: list[Snapshot]) -> None:
    for ch, label in [
        ("rc_out_ch5", "servo5"),
        ("rc_out_ch6", "servo6"),
        ("rc_out_ch7", "servo7"),
        ("rc_out_ch8", "servo8"),
    ]:
        axis.step(t, [getattr(r, ch) for r in rows], where="post", label=label)
    axis.axhline(RC_NEUTRAL, color="black", linewidth=0.8, alpha=0.5)
    heave_axis = axis.twinx()
    heave_axis.step(
        t,
        [r.rc_out_vertical_plant_cmd_norm for r in rows],
        where="post",
        label="MuJoCo plant vertical cmd",
        color="tab:red",
        linewidth=1.5,
    )
    heave_axis.set_ylabel("plant cmd norm")
    axis.set_title("Vertical servo output")
    axis.set_ylabel("PWM us")
    axis.set_xlabel("time [s]")
    pwm_lines, pwm_labels = axis.get_legend_handles_labels()
    heave_lines, heave_labels = heave_axis.get_legend_handles_labels()
    axis.legend(pwm_lines + heave_lines, pwm_labels + heave_labels, loc="best", fontsize=8)


__all__ = ["write_plot"]
