"""State initialization for the UUV control GUI application."""

from __future__ import annotations

from .config import (
    WINDOW_GEOMETRY,
    WINDOW_MINSIZE,
)
from .app_state_vars import initialize_gui_vars
from .models import RcReplaySample
from .runtime import MultiThreadedExecutor, subprocess, threading, tk


def initialize_gui_app_state(self, node, title: str) -> None:
    self.node = node
    self.root = tk.Tk()
    self.root.title(title)
    self.root.geometry(WINDOW_GEOMETRY)
    self.root.minsize(*WINDOW_MINSIZE)

    self._executor = MultiThreadedExecutor(num_threads=2)
    self._executor.add_node(self.node)
    self._spin_thread = threading.Thread(target=self._spin, daemon=True)
    self._spin_thread.start()

    initialize_gui_vars(self)
    initialize_runtime_state(self)
    initialize_widget_refs(self)


def initialize_runtime_state(self) -> None:
    self._last_event_top = ""
    self._last_vehicle_info_wall = 0.0
    self._last_sim_stack_probe_wall = -1.0
    self._external_sim_stack_running_cached = False
    self._sim_stack_owned_by_gui = False
    self._guided_control_prev = False
    self._rc_override_prev = False
    self._pilot_input_release_requested = False
    self._rc_replay_samples: list[RcReplaySample] = []
    self._rc_replay_thread: threading.Thread | None = None
    self._rc_replay_duration_s = 0.0
    self._rc_replay_slider_dragging = False
    self._rc_replay_seek_lock = threading.Lock()
    self._rc_replay_seek_time_s: float | None = None
    self._rc_replay_stop_event = threading.Event()
    self._rc_replay_pause_event = threading.Event()
    self._sim_stack_process: subprocess.Popen[str] | None = None
    self._ros_pkg_process: subprocess.Popen[str] | None = None
    self._ros_pkg_thread: threading.Thread | None = None
    self._ros_build_process: subprocess.Popen[str] | None = None
    self._ros_build_thread: threading.Thread | None = None
    self._rviz_process: subprocess.Popen[str] | None = None
    self._rviz_thread: threading.Thread | None = None
    self._ping360_view_process: subprocess.Popen[str] | None = None
    self._ping360_view_log_path = None
    self._sim_stack_thread: threading.Thread | None = None
    self._sim_stack_reset_thread: threading.Thread | None = None
    self._sim_stack_log_path = None
    self._closed = False
    self._after_id: str | None = None
    self._rc_fast_after_id: str | None = None


def initialize_widget_refs(self) -> None:
    self.ping360_window = None
    self.main_container = None
    self.telemetry_panel = None
    self.control_panel = None
    self.control_tools_panel = None
    self.control_tools_content = None
    self.control_tools_toggle_button = None
    self.ros2_panel_frame = None
    self.ros2_panel_button = None
    self.mavros_toggle_button = None
    self.rviz_toggle_button = None
    self.sim_stack_start_button = None
    self.sim_stack_stop_button = None
    self.sim_launch_preset_combo = None
    self.command_ready_label = None
    self.physics_window = None
    self.physics_canvas = None
    self.physics_scroll_frame = None
    self.buoy_layout_window = None
    self.buoy_layout_canvas = None
    self.buoy_layout_button = None
    self.buoy_layout_editor = None
