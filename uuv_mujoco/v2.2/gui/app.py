"""Tk application entry point for the MuJoCo UUV control GUI."""

from __future__ import annotations

from .autotune_mixin import AutoTuneMixin
from .config import (
    BACKEND_AUTO,
    BACKEND_MAVROS,
    BACKEND_NONE,
    BACKEND_SIM_BRIDGE,
    DEFAULT_AUTOTUNE_BAG,
    DEFAULT_RC_REPLAY_BAG,
    PHYSICS_PARAM_SPECS,
    ROS_PACKAGE_DEFAULT_FCU_URL,
    UI_UPDATE_PERIOD_MS,
    WINDOW_GEOMETRY,
    WINDOW_MINSIZE,
)
from .control_display_mixin import ControlDisplayMixin
from .layout_mixin import LayoutMixin
from .models import RcReplaySample
from .node import UuvGuiNode
from .physics_mixin import PhysicsMixin
from .replay_mixin import RcReplayMixin
from .ros_process_mixin import RosProcessMixin
from .runtime import MultiThreadedExecutor, argparse, rclpy, subprocess, threading, tk


class UuvControlGui(
    RcReplayMixin,
    ControlDisplayMixin,
    RosProcessMixin,
    PhysicsMixin,
    AutoTuneMixin,
    LayoutMixin,
):
    MODE_BUTTONS = ("MANUAL", "STABILIZE", "ALT_HOLD", "GUIDED", "SURFACE", "POSHOLD")

    def __init__(self, node: UuvGuiNode, title: str):
        self.node = node
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry(WINDOW_GEOMETRY)
        self.root.minsize(*WINDOW_MINSIZE)

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self.control_enabled = tk.BooleanVar(value=False)
        self.rc_override_enabled = tk.BooleanVar(value=False)
        self.control_details_visible = tk.BooleanVar(value=False)
        self.vehicle_details_visible = tk.BooleanVar(value=False)
        self.telemetry_visible = tk.BooleanVar(value=True)
        self.ros2_panel_visible = tk.BooleanVar(value=False)
        self.autotune_visible = tk.BooleanVar(value=False)
        self.forward_var = tk.DoubleVar(value=0.0)
        self.lateral_var = tk.DoubleVar(value=0.0)
        self.heave_var = tk.DoubleVar(value=0.0)
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.rc_forward_var = tk.DoubleVar(value=0.0)
        self.rc_lateral_var = tk.DoubleVar(value=0.0)
        self.rc_heave_var = tk.DoubleVar(value=0.0)
        self.rc_yaw_var = tk.DoubleVar(value=0.0)
        self.vehicle_summary_var = tk.StringVar(value="vehicle: disconnected")
        self.motion_summary_var = tk.StringVar(value="motion: n/a")
        self.depth_target_var = tk.StringVar(value="n/a")
        self.depth_source_var = tk.StringVar(value="depth source: unavailable")
        self.mode_var = tk.StringVar(value="mode: UNKNOWN")
        self.status_var = tk.StringVar(value="disconnected")
        self.command_ready_var = tk.StringVar(value="WAIT: vehicle")
        self.battery_var = tk.StringVar(value="battery: n/a")
        self.pose_var = tk.StringVar(value="pose: n/a")
        self.vel_var = tk.StringVar(value="velocity: n/a")
        self.imu_var = tk.StringVar(value="imu: n/a")
        self.autopilot_var = tk.StringVar(value="autopilot: n/a")
        self.age_var = tk.StringVar(value="state age: n/a")
        self.control_summary_var = tk.StringVar(value="control: idle")
        self.rc_override_var = tk.StringVar(value="pilot input: off")
        self.control_var = tk.StringVar(value="setpoint: x=0.00 y=0.00 z=0.00 yaw=0.00")
        self.rc_replay_path_var = tk.StringVar(value=str(DEFAULT_RC_REPLAY_BAG))
        self.rc_replay_rate_var = tk.StringVar(value="1.0")
        self.rc_replay_status_var = tk.StringVar(value="replay: unloaded")
        self.rc_replay_position_var = tk.DoubleVar(value=0.0)
        self.rc_replay_time_var = tk.StringVar(value="00:00.0 / 00:00.0")
        self.physics_status_var = tk.StringVar(value="physics params: idle")
        self.physics_param_vars = {
            str(spec["key"]): tk.StringVar(value="")
            for spec in PHYSICS_PARAM_SPECS
        }
        self.autotune_bag_var = tk.StringVar(value=str(DEFAULT_AUTOTUNE_BAG))
        self.autotune_start_var = tk.StringVar(value="60")
        self.autotune_duration_var = tk.StringVar(value="120")
        self.autotune_candidates_var = tk.StringVar(value="15")
        # Legacy polynomial/gain tuned mode used "0.58".
        self.autotune_servo_scale_var = tk.StringVar(value="1.0")
        self.autotune_mode_var = tk.StringVar(value="plant-rc-out")
        self.autotune_candidate_set_var = tk.StringVar(value="ellipsoid5")
        self.autotune_status_var = tk.StringVar(value="autotune: idle")
        self.autotune_out_dir_var = tk.StringVar(value="")
        self.autotune_apply_best_var = tk.BooleanVar(value=False)
        self.sim_stack_status_var = tk.StringVar(value="sim: stopped")
        self.ros_pkg_status_var = tk.StringVar(value="mavros: stopped")
        self.ros_pkg_fcu_url_var = tk.StringVar(value=ROS_PACKAGE_DEFAULT_FCU_URL)
        self.rviz_status_var = tk.StringVar(value="rviz: stopped")
        self.ping360_view_status_var = tk.StringVar(value="ping360 view: closed")
        self.ping360_enabled_var = tk.BooleanVar(value=False)
        self.ping360_range_var = tk.StringVar(value="2.0")
        self.ping360_num_steps_var = tk.StringVar(value="1")
        self.ping360_gain_var = tk.StringVar(value="0")
        self.ping360_interface_var = tk.StringVar(value="ethernet")
        self.ping360_frequency_var = tk.StringVar(value="750")
        self.ping360_start_angle_var = tk.StringVar(value="0")
        self.ping360_stop_angle_var = tk.StringVar(value="399")
        self.ping360_summary_var = tk.StringVar(value="ping360: no status")

        self._last_event_top = ""
        self._last_vehicle_info_wall = 0.0
        self._last_sim_stack_probe_wall = -1.0
        self._external_sim_stack_running_cached = False
        self._sim_stack_owned_by_gui = False
        self._guided_control_prev = False
        self._rc_override_prev = False
        self._pilot_input_release_requested = False
        self._rc_replay_samples: list[RcReplaySample] = []
        self._rc_replay_thread: Optional[threading.Thread] = None
        self._rc_replay_duration_s = 0.0
        self._rc_replay_slider_dragging = False
        self._rc_replay_seek_lock = threading.Lock()
        self._rc_replay_seek_time_s: Optional[float] = None
        self._rc_replay_stop_event = threading.Event()
        self._rc_replay_pause_event = threading.Event()
        self._autotune_process: subprocess.Popen[str] | None = None
        self._autotune_thread: threading.Thread | None = None
        self._sim_stack_process: subprocess.Popen[str] | None = None
        self._ros_pkg_process: subprocess.Popen[str] | None = None
        self._ros_pkg_thread: threading.Thread | None = None
        self._ros_build_process: subprocess.Popen[str] | None = None
        self._ros_build_thread: threading.Thread | None = None
        self._rviz_process: subprocess.Popen[str] | None = None
        self._rviz_thread: threading.Thread | None = None
        self._ping360_view_process: subprocess.Popen[str] | None = None
        self._ping360_view_log_path: Path | None = None
        self.ping360_window: tk.Toplevel | None = None
        self._sim_stack_thread: threading.Thread | None = None
        self._sim_stack_reset_thread: threading.Thread | None = None
        self._sim_stack_log_path: Path | None = None
        self.autotune_monitor_window: tk.Toplevel | None = None
        self.autotune_monitor_status_var = tk.StringVar(value="autotune monitor: idle")
        self.autotune_monitor_progress_var = tk.DoubleVar(value=0.0)
        self.autotune_tree: ttk.Treeview | None = None
        self.autotune_chart_canvas: tk.Canvas | None = None
        self.autotune_log_text: tk.Text | None = None
        self._autotune_candidate_order: list[str] = []
        self._autotune_candidate_rows: dict[str, dict[str, Any]] = {}
        self._autotune_current_candidate: str | None = None
        self._closed = False
        self._after_id: Optional[str] = None
        self.main_container: ttk.Frame | None = None
        self.telemetry_panel: ttk.Frame | None = None
        self.control_panel: ttk.Frame | None = None
        self.ros2_panel_frame: ttk.LabelFrame | None = None
        self.ros2_panel_button: ttk.Button | None = None
        self.mavros_toggle_button: ttk.Button | None = None
        self.rviz_toggle_button: ttk.Button | None = None
        self.sim_stack_start_button: ttk.Button | None = None
        self.sim_stack_stop_button: ttk.Button | None = None
        self.command_ready_label: ttk.Label | None = None
        self.physics_window: tk.Toplevel | None = None
        self.physics_canvas: tk.Canvas | None = None
        self.physics_scroll_frame: ttk.Frame | None = None
        self.autotune_toggle_button: ttk.Button | None = None
        self.autotune_frame: ttk.LabelFrame | None = None

        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._schedule_update()

    def _spin(self) -> None:
        self._executor.spin()

    def _schedule_update(self) -> None:
        if not self._closed and self.root.winfo_exists():
            self._after_id = self.root.after(UI_UPDATE_PERIOD_MS, self._update_ui)
    def _on_close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self._rc_replay_stop_event.set()
        self._rc_replay_pause_event.clear()
        self._stop_ping360_view()
        self._terminate_process_group(self._ros_pkg_process)
        self._terminate_process_group(self._ros_build_process)
        self._terminate_process_group(self._rviz_process)
        owned_sim_stack = self._sim_stack_owned_by_gui or (
            self._sim_stack_process is not None
            and self._sim_stack_process.poll() is None
        )
        self._terminate_sim_stack_process()
        if owned_sim_stack:
            self._reset_sim_stack_blocking()
        if self._autotune_process is not None and self._autotune_process.poll() is None:
            try:
                self._terminate_process_group(self._autotune_process)
            except Exception:
                pass
        try:
            self.node.publish_rc_release()
        except Exception:
            pass
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UUV MAVROS telemetry and control GUI")
    parser.add_argument(
        "--namespace",
        default="/mavros",
        help="MAVROS namespace to use (default: /mavros)",
    )
    parser.add_argument(
        "--backend",
        choices=(BACKEND_AUTO, BACKEND_NONE, BACKEND_MAVROS, BACKEND_SIM_BRIDGE, "sim"),
        default=BACKEND_AUTO,
        help="Control/RC compatibility profile (default: auto)",
    )
    parser.add_argument(
        "--title",
        default="UUV Control GUI",
        help="GUI window title",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = UuvGuiNode(namespace=args.namespace, backend=args.backend)
    app = UuvControlGui(node=node, title=args.title)
    app.run()
    return 0
