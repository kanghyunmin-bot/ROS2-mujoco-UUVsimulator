#!/usr/bin/env python3
"""Record ALT_HOLD closed-loop signals on one timeline.

This tool is intentionally read-only. It does not publish commands, change
parameters, or touch ArduPilot. It records the signals needed to separate:

* GUI MANUAL_CONTROL authority
* RC override authority
* Bar30/depth and vertical velocity sign
* ArduSub servo output
* MuJoCo plant drift
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path


RC_MIN = 1100
RC_MAX = 1900
RC_NEUTRAL = 1500
RC3_TRIM = 1100
JS_GAIN_DEFAULT = 0.1
JS_GAIN_MIN = 0.25
JS_GAIN_MAX = 2.0
JS_GAIN_STEPS = 4
JS_THR_GAIN = 1.0
PILOT_SPEED_UP_CM_S = 100.0
PILOT_SPEED_DN_CM_S = 0.0
RC3_DZ = 30


@dataclass
class Snapshot:
    t_s: float

    manual_x: float = math.nan
    manual_y: float = math.nan
    manual_z: float = math.nan
    manual_r: float = math.nan
    manual_expected_rc3: float = math.nan
    manual_expected_althold_climb_cm_s: float = math.nan

    rc_in_ch3: float = math.nan
    rc_in_source: str = "none"

    depth_bar30_m: float = math.nan
    pressure_bar30_pa: float = math.nan
    mavros_local_depth_m: float = math.nan
    mavros_velz_down_mps: float = math.nan
    dvl_velz_down_mps: float = math.nan

    rc_out_ch5: float = math.nan
    rc_out_ch6: float = math.nan
    rc_out_ch7: float = math.nan
    rc_out_ch8: float = math.nan
    rc_out_vertical_mean: float = math.nan
    rc_out_vertical_span: float = math.nan
    rc_out_vertical_plant_cmd_norm: float = math.nan

    mujoco_depth_m: float = math.nan
    mujoco_depth_rate_down_mps: float = math.nan
    sim_odom_depth_m: float = math.nan
    sim_odom_depth_rate_down_mps: float = math.nan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def effective_js_gain() -> float:
    """Mirror ArduSub joystick.cpp init_joystick() gain selection."""
    steps = max(1, int(JS_GAIN_STEPS))
    if steps == 1 or (JS_GAIN_DEFAULT < JS_GAIN_MAX + 0.01 and JS_GAIN_DEFAULT > JS_GAIN_MIN - 0.01):
        gain = clamp(JS_GAIN_DEFAULT, JS_GAIN_MIN, JS_GAIN_MAX)
    else:
        gain = JS_GAIN_MIN + (steps / 2.0 - 1.0) * (JS_GAIN_MAX - JS_GAIN_MIN) / float(steps - 1)
    return clamp(gain, 0.1, 1.0)


def manual_heave_to_expected_rc3(heave: float) -> float:
    """Mirror ArduSub joystick.cpp MANUAL_CONTROL.z -> RC3 override.

    The GUI publishes normalized heave [-1, +1] on /mavros/manual_control/send.
    sitl_transport converts that to MAVLink MANUAL_CONTROL.z [0, 1000].
    ArduSub then applies the effective joystick gain from init_joystick()
    and JS_THR_GAIN.
    """
    heave = clamp(float(heave), -1.0, 1.0)
    manual_z = 500.0 + heave * 500.0
    gain = effective_js_gain()
    throttle_scale = 0.8 * gain * JS_THR_GAIN
    throttle_base = RC_NEUTRAL - 500.0 * throttle_scale
    return clamp(manual_z * throttle_scale + throttle_base, RC_MIN, RC_MAX)


def rc3_to_expected_althold_climb(rc3_pwm: float) -> float:
    """Approximate level-vehicle ALT_HOLD target climb rate from RC3.

    This follows the local ArduSub code path:
    control_althold.cpp uses channel_throttle->norm_input(), which is trim
    based, then get_pilot_desired_climb_rate() uses the RC min/max midpoint.
    """
    rc3_pwm = float(rc3_pwm)
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
    speed_dn = abs(PILOT_SPEED_DN_CM_S) if PILOT_SPEED_DN_CM_S != 0.0 else abs(PILOT_SPEED_UP_CM_S)
    return clamp(target, -speed_dn, PILOT_SPEED_UP_CM_S)


class AltHoldDiagnosticsLogger:
    def __init__(self, args: argparse.Namespace) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped, TwistStamped
            from mavros_msgs.msg import ManualControl, RCIn, RCOut
            from nav_msgs.msg import Odometry
            from rclpy.node import Node
            from std_msgs.msg import Float32
        except ModuleNotFoundError as exc:
            raise SystemExit(
                "ROS2/mavros Python modules are not available. Source the same "
                "ROS2 environment used by the simulator, then rerun this tool."
            ) from exc

        self.rclpy = rclpy
        self.PoseStamped = PoseStamped
        self.TwistStamped = TwistStamped
        self.ManualControl = ManualControl
        self.RCIn = RCIn
        self.RCOut = RCOut
        self.Odometry = Odometry
        self.Float32 = Float32

        self.duration_s = float(args.duration)
        self.sample_hz = float(args.sample_hz)
        self.plot = bool(args.plot)
        self.output_dir = Path(args.output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_dir / f"althold_diag_{stamp}.csv"
        self.plot_path = self.output_dir / f"althold_diag_{stamp}.png"

        rclpy.init(args=None)
        self.node = Node("althold_diagnostics_logger")
        self.state = Snapshot(t_s=0.0)
        self.rows: list[Snapshot] = []
        self.done = False
        self.start_wall = time.monotonic()
        self.last_mujoco_depth_t: float | None = None
        self.last_mujoco_depth_m: float | None = None
        self.last_sim_odom_t: float | None = None
        self.last_sim_odom_depth_m: float | None = None

        q = 20
        self.node.create_subscription(ManualControl, "/mavros/manual_control/send", self._on_manual, q)
        self.node.create_subscription(RCIn, "/mavros/rc/in", self._on_rc_in, q)
        self.node.create_subscription(RCOut, "/mavros/rc/out", self._on_rc_out, q)
        self.node.create_subscription(Float32, "/depth", self._on_depth, q)
        self.node.create_subscription(Float32, "/bar30/pressure_pa", self._on_pressure, q)
        self.node.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_mavros_pose, q)
        self.node.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._on_mavros_vel, q)
        self.node.create_subscription(TwistStamped, "/dvl/velocity", self._on_dvl_vel, q)
        self.node.create_subscription(PoseStamped, "/mujoco/ground_truth/pose", self._on_mujoco_pose, q)
        self.node.create_subscription(Odometry, "/sim/odom", self._on_sim_odom, q)
        self.node.create_timer(1.0 / max(self.sample_hz, 1.0), self._sample)

    def _elapsed(self) -> float:
        return time.monotonic() - self.start_wall

    def _on_manual(self, msg) -> None:
        self.state.manual_x = float(getattr(msg, "x", math.nan))
        self.state.manual_y = float(getattr(msg, "y", math.nan))
        self.state.manual_z = float(getattr(msg, "z", math.nan))
        self.state.manual_r = float(getattr(msg, "r", math.nan))
        if math.isfinite(self.state.manual_z):
            rc3 = manual_heave_to_expected_rc3(self.state.manual_z)
            self.state.manual_expected_rc3 = rc3
            self.state.manual_expected_althold_climb_cm_s = rc3_to_expected_althold_climb(rc3)

    def _on_rc_in(self, msg) -> None:
        channels = list(getattr(msg, "channels", []))
        if len(channels) >= 3:
            self.state.rc_in_ch3 = float(channels[2])
            self.state.rc_in_source = "rc_override_mirror"

    def _on_rc_out(self, msg) -> None:
        channels = list(getattr(msg, "channels", []))
        if len(channels) >= 8:
            vertical = [float(channels[idx]) for idx in (4, 5, 6, 7)]
            self.state.rc_out_ch5 = vertical[0]
            self.state.rc_out_ch6 = vertical[1]
            self.state.rc_out_ch7 = vertical[2]
            self.state.rc_out_ch8 = vertical[3]
            self.state.rc_out_vertical_mean = sum(vertical) / 4.0
            self.state.rc_out_vertical_span = max(vertical) - min(vertical)
            # Raw PWM average is not a heave command because real MOT_5/8 are
            # reversed. Convert final SERVO_OUTPUT_RAW deltas back into the
            # MuJoCo actuator command sign convention used by thruster_mapping.
            signs = (-1.0, 1.0, 1.0, -1.0)
            self.state.rc_out_vertical_plant_cmd_norm = sum(
                ((pwm - RC_NEUTRAL) / 400.0) * sign
                for pwm, sign in zip(vertical, signs)
            ) / 4.0

    def _on_depth(self, msg) -> None:
        self.state.depth_bar30_m = float(getattr(msg, "data", math.nan))

    def _on_pressure(self, msg) -> None:
        self.state.pressure_bar30_pa = float(getattr(msg, "data", math.nan))

    def _on_mavros_pose(self, msg) -> None:
        self.state.mavros_local_depth_m = max(0.0, -float(msg.pose.position.z))

    def _on_mavros_vel(self, msg) -> None:
        # Bridge publishes ENU local velocity; ArduSub VELZ convention is down positive.
        self.state.mavros_velz_down_mps = -float(msg.twist.linear.z)

    def _on_dvl_vel(self, msg) -> None:
        # /dvl/velocity is base_link FLU in the current bridge; down is -z.
        self.state.dvl_velz_down_mps = -float(msg.twist.linear.z)

    def _on_mujoco_pose(self, msg) -> None:
        now = self._elapsed()
        depth_m = max(0.0, -float(msg.pose.position.z))
        self.state.mujoco_depth_m = depth_m
        if self.last_mujoco_depth_t is not None and self.last_mujoco_depth_m is not None:
            dt = now - self.last_mujoco_depth_t
            if 1.0e-4 <= dt <= 1.0:
                self.state.mujoco_depth_rate_down_mps = (depth_m - self.last_mujoco_depth_m) / dt
        self.last_mujoco_depth_t = now
        self.last_mujoco_depth_m = depth_m

    def _on_sim_odom(self, msg) -> None:
        now = self._elapsed()
        depth_m = max(0.0, -float(msg.pose.pose.position.z))
        self.state.sim_odom_depth_m = depth_m
        if self.last_sim_odom_t is not None and self.last_sim_odom_depth_m is not None:
            dt = now - self.last_sim_odom_t
            if 1.0e-4 <= dt <= 1.0:
                self.state.sim_odom_depth_rate_down_mps = (depth_m - self.last_sim_odom_depth_m) / dt
        self.last_sim_odom_t = now
        self.last_sim_odom_depth_m = depth_m

    def _sample(self) -> None:
        if self.done:
            return
        t_s = self._elapsed()
        row = Snapshot(**asdict(self.state))
        row.t_s = t_s
        self.rows.append(row)
        if t_s >= self.duration_s:
            self._finish()

    def _finish(self) -> None:
        if self.done:
            return
        self.done = True
        self._write_csv()
        if self.plot:
            self._write_plot()
        self._print_summary()

    def _write_csv(self) -> None:
        if not self.rows:
            return
        with self.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(asdict(self.rows[0]).keys()))
            writer.writeheader()
            for row in self.rows:
                writer.writerow(asdict(row))

    @staticmethod
    def _finite(values: list[float]) -> list[float]:
        return [float(v) for v in values if math.isfinite(float(v))]

    def _write_plot(self) -> None:
        try:
            import matplotlib.pyplot as plt
        except ModuleNotFoundError:
            print("[althold_diag] matplotlib not available; skipped plot", file=sys.stderr)
            return
        if not self.rows:
            return
        t = [r.t_s for r in self.rows]
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

        axes[0].plot(t, [r.manual_z for r in self.rows], label="MANUAL_CONTROL.z", color="tab:blue")
        axes[0].plot(t, [r.manual_expected_rc3 for r in self.rows], label="expected RC3 from manual", color="tab:green")
        axes[0].plot(t, [r.rc_in_ch3 for r in self.rows], label="/mavros/rc/in ch3", color="tab:gray")
        axes[0].set_title("Pilot input")
        axes[0].set_ylabel("axis / PWM")
        axes[0].legend(loc="best", fontsize=8)

        axes[1].plot(t, [r.depth_bar30_m for r in self.rows], label="/depth Bar30", color="tab:blue")
        axes[1].plot(t, [r.mujoco_depth_m for r in self.rows], label="MuJoCo depth", color="tab:red")
        axes[1].plot(t, [r.sim_odom_depth_m for r in self.rows], label="/sim/odom depth", color="tab:orange")
        axes[1].set_title("Depth")
        axes[1].set_ylabel("m down")
        axes[1].legend(loc="best", fontsize=8)

        axes[2].plot(t, [r.mavros_velz_down_mps for r in self.rows], label="local velocity z down", color="tab:purple")
        axes[2].plot(t, [r.dvl_velz_down_mps for r in self.rows], label="DVL z down", color="tab:brown")
        axes[2].plot(t, [r.mujoco_depth_rate_down_mps for r in self.rows], label="MuJoCo dz/dt down", color="tab:red")
        axes[2].set_title("Vertical velocity")
        axes[2].set_ylabel("m/s down")
        axes[2].legend(loc="best", fontsize=8)

        for ch, label in [
            ("rc_out_ch5", "servo5"),
            ("rc_out_ch6", "servo6"),
            ("rc_out_ch7", "servo7"),
            ("rc_out_ch8", "servo8"),
        ]:
            axes[3].plot(t, [getattr(r, ch) for r in self.rows], label=label)
        axes[3].axhline(1500, color="black", linewidth=0.8, alpha=0.5)
        heave_axis = axes[3].twinx()
        heave_axis.plot(
            t,
            [r.rc_out_vertical_plant_cmd_norm for r in self.rows],
            label="MuJoCo plant vertical cmd",
            color="tab:red",
            linewidth=1.5,
        )
        heave_axis.set_ylabel("plant cmd norm")
        axes[3].set_title("Vertical servo output")
        axes[3].set_ylabel("PWM us")
        axes[3].set_xlabel("time [s]")
        pwm_lines, pwm_labels = axes[3].get_legend_handles_labels()
        heave_lines, heave_labels = heave_axis.get_legend_handles_labels()
        axes[3].legend(pwm_lines + heave_lines, pwm_labels + heave_labels, loc="best", fontsize=8)

        fig.tight_layout()
        fig.savefig(self.plot_path, dpi=150)
        plt.close(fig)

    def _print_summary(self) -> None:
        rows = self.rows
        rc3 = self._finite([r.rc_in_ch3 for r in rows])
        expected_rc3 = self._finite([r.manual_expected_rc3 for r in rows])
        expected_climb = self._finite([r.manual_expected_althold_climb_cm_s for r in rows])
        depth = self._finite([r.mujoco_depth_m for r in rows])
        rate = self._finite([r.mujoco_depth_rate_down_mps for r in rows])
        servo_mean = self._finite([r.rc_out_vertical_mean for r in rows])
        plant_cmd = self._finite([r.rc_out_vertical_plant_cmd_norm for r in rows])

        print(f"[althold_diag] wrote CSV: {self.csv_path}")
        if self.plot and self.plot_path.exists():
            print(f"[althold_diag] wrote plot: {self.plot_path}")
        if expected_rc3:
            print(
                "[althold_diag] manual expected RC3 range: "
                f"{min(expected_rc3):.1f}..{max(expected_rc3):.1f} PWM"
            )
        if expected_climb:
            print(
                "[althold_diag] manual expected ALT_HOLD climb range: "
                f"{min(expected_climb):+.1f}..{max(expected_climb):+.1f} cm/s"
            )
        if rc3:
            print(f"[althold_diag] /mavros/rc/in ch3 range: {min(rc3):.1f}..{max(rc3):.1f} PWM")
            print("[althold_diag] note: /mavros/rc/in is an RC override mirror in this bridge, not MANUAL_CONTROL's internal ArduSub RC3.")
        if servo_mean:
            print(
                "[althold_diag] vertical servo mean range: "
                f"{min(servo_mean):.1f}..{max(servo_mean):.1f} PWM"
            )
        if plant_cmd:
            print(
                "[althold_diag] MuJoCo vertical plant command range: "
                f"{min(plant_cmd):+.3f}..{max(plant_cmd):+.3f}"
            )
        if len(depth) >= 2:
            print(f"[althold_diag] MuJoCo depth drift: {depth[-1] - depth[0]:+.3f} m")
        if rate:
            rms = math.sqrt(sum(v * v for v in rate) / len(rate))
            print(f"[althold_diag] MuJoCo vertical-rate RMS: {rms:.4f} m/s")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0, help="Recording duration in seconds.")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="CSV sampling rate.")
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parents[1] / "logs" / "diagnostics"),
        help="Directory for CSV/plot outputs.",
    )
    parser.add_argument("--no-plot", dest="plot", action="store_false", help="Skip PNG plot generation.")
    parser.set_defaults(plot=True)
    return parser.parse_args()


def main() -> None:
    logger = AltHoldDiagnosticsLogger(parse_args())
    try:
        while logger.rclpy.ok() and not logger.done:
            logger.rclpy.spin_once(logger.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        logger._finish()
    finally:
        if not logger.done:
            logger._finish()
        try:
            logger.node.destroy_node()
        except Exception:
            pass
        try:
            logger.rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
