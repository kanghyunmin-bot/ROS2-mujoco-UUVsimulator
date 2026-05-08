"""ROS, RViz, Ping360, and simulator process controls for the GUI."""

from __future__ import annotations

import datetime as _dt

from .config import *
from .helpers import *
from .models import ControlCommands, RcReplaySample
from .node import UuvGuiNode
from .ros_tools import *
from .runtime import *
from .widgets import VirtualJoystick

class RosProcessMixin:
    def _toggle_ros2_panel(self) -> None:
        show = not self.ros2_panel_visible.get()
        self.ros2_panel_visible.set(show)
        if self.ros2_panel_frame is None:
            return
        if show:
            self.ros2_panel_frame.grid()
            if self.ros2_panel_button is not None:
                self.ros2_panel_button.config(text="Hide ROS2")
        else:
            self.ros2_panel_frame.grid_remove()
            if self.ros2_panel_button is not None:
                self.ros2_panel_button.config(text="ROS2 Panel")

    def _ros_pkg_running(self) -> bool:
        return self._ros_pkg_process is not None and self._ros_pkg_process.poll() is None

    def _ros_build_running(self) -> bool:
        return self._ros_build_process is not None and self._ros_build_process.poll() is None

    def _rviz_running(self) -> bool:
        return self._rviz_process is not None and self._rviz_process.poll() is None

    def _set_ros_pkg_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.ros_pkg_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.ros_pkg_status_var.set(text))
        except Exception:
            pass

    def _set_rviz_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.rviz_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.rviz_status_var.set(text))
        except Exception:
            pass

    def _refresh_ros2_buttons(self) -> None:
        if self.mavros_toggle_button is not None:
            self.mavros_toggle_button.config(text="MAVROS OFF" if self._ros_pkg_running() else "MAVROS ON")
        if self.rviz_toggle_button is not None:
            self.rviz_toggle_button.config(text="RViz OFF" if self._rviz_running() else "RViz ON")

    @staticmethod
    def _terminate_process_group(proc: subprocess.Popen[str] | None) -> None:
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass

    def _start_logged_ros_process(
        self,
        *,
        cmd: list[str],
        label: str,
        log_prefix: str,
        attr_name: str,
        status_callback,
    ) -> None:
        log_dir = SIM_STACK_DIR / "logs"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"{log_prefix}_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            log_file = log_path.open("w", encoding="utf-8")
        except Exception as exc:
            status_callback(f"{label}: log open failed: {exc}")
            return

        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(ROS_WORKSPACE_DIR),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except Exception as exc:
            log_file.close()
            status_callback(f"{label}: start failed: {exc}")
            return
        finally:
            try:
                log_file.close()
            except Exception:
                pass

        setattr(self, attr_name, proc)
        status_callback(f"{label}: starting ({log_path.name})")
        self.node.push_event(f"{label} start requested: {log_path.name}")
        thread = threading.Thread(
            target=self._watch_ros_process,
            args=(proc, log_path, label, attr_name, status_callback),
            daemon=True,
        )
        if attr_name == "_ros_pkg_process":
            self._ros_pkg_thread = thread
        elif attr_name == "_ros_build_process":
            self._ros_build_thread = thread
        elif attr_name == "_rviz_process":
            self._rviz_thread = thread
        thread.start()
        self._refresh_ros2_buttons()

    def _restart_sim_stack_after_mavros_mode_change(self, *, want_mavros_running: bool, reason: str) -> None:
        if not self._sim_stack_running():
            return
        self._set_sim_stack_status(f"sim: restarting for {reason}")
        self.node.push_event(f"sim restart requested for {reason}")
        self._terminate_sim_stack_process()

        def start_when_ready() -> None:
            if self._closed:
                return
            if self._sim_stack_running():
                self.root.after(500, start_when_ready)
                return
            if self._ros_pkg_running() != want_mavros_running:
                self.root.after(500, start_when_ready)
                return
            self._start_sim_stack()

        self.root.after(1500, start_when_ready)

    def _watch_ros_process(
        self,
        proc: subprocess.Popen[str],
        log_path: Path,
        label: str,
        attr_name: str,
        status_callback,
    ) -> None:
        last_line = ""
        try:
            with log_path.open("r", encoding="utf-8", errors="replace") as log_stream:
                while True:
                    raw_line = log_stream.readline()
                    if raw_line:
                        line = raw_line.strip()
                        if not line:
                            continue
                        last_line = line
                        if line.startswith(("[INFO]", "[WARN]", "[ERROR]", "[rviz", "[ros2", "[colcon")):
                            short_line = line if len(line) <= 150 else f"{line[:147]}..."
                            self.node.push_event(f"{label}: {short_line}")
                        continue
                    if proc.poll() is not None:
                        break
                    time.sleep(0.1)
                for raw_line in log_stream:
                    line = raw_line.strip()
                    if line:
                        last_line = line
            rc = proc.returncode if proc.returncode is not None else proc.wait()
        except Exception as exc:
            rc = -1
            last_line = f"reader failed: {exc}"

        def finish() -> None:
            if getattr(self, attr_name) is proc:
                setattr(self, attr_name, None)
            if rc == 0:
                status_callback(f"{label}: exited")
                self.node.push_event(f"{label} exited")
            elif rc < 0:
                status_callback(f"{label}: stopped")
                self.node.push_event(f"{label} stopped")
            else:
                text = last_line if last_line else f"rc={rc}"
                status_callback(f"{label} failed: {text}")
                self.node.push_event(f"{label} failed: {text}")
            self._refresh_ros2_buttons()

        try:
            self.root.after(0, finish)
        except Exception:
            pass

    def _build_ros_pkg(self) -> None:
        if self._ros_build_running():
            self._set_ros_pkg_status("ros2 build: already running")
            return
        if not ROS_PACKAGE_DIR.exists():
            self._set_ros_pkg_status(f"ros2 build: package missing: {ROS_PACKAGE_DIR}")
            return
        base_paths = []
        if PING360_MSG_PACKAGE_DIR.exists():
            base_paths.append("ping360_sonar_msgs")
        base_paths.append("kmu26_auv")
        command = (
            "colcon build --base-paths "
            + " ".join(shlex.quote(path) for path in base_paths)
            + " --symlink-install --cmake-clean-cache"
        )
        self._start_logged_ros_process(
            cmd=ros_bash_command(command, include_workspace=False),
            label="ros2 build",
            log_prefix="gui_ros2_build",
            attr_name="_ros_build_process",
            status_callback=self._set_ros_pkg_status,
        )

    def _toggle_ros_pkg_stack(self) -> None:
        if self._ros_pkg_running():
            sim_was_running = self._sim_stack_running()
            self._terminate_process_group(self._ros_pkg_process)
            self._set_ros_pkg_status("mavros: stopping")
            self._refresh_ros2_buttons()
            if sim_was_running:
                self._restart_sim_stack_after_mavros_mode_change(
                    want_mavros_running=False,
                    reason="MAVROS OFF",
                )
            return
        self._start_ros_pkg_stack()

    def _start_ros_pkg_stack(self) -> None:
        if self._ros_build_running():
            self._set_ros_pkg_status("mavros: wait for package build to finish")
            return
        fcu_url = self.ros_pkg_fcu_url_var.get().strip() or ROS_PACKAGE_DEFAULT_FCU_URL
        self.ros_pkg_fcu_url_var.set(fcu_url)
        # Simulation path: keep rospkg/kmu26_auv read-only and attach MAVROS
        # directly to ArduSub/SITL. Start the simulator in real-pkg-compat mode
        # so the bridge keeps the simulated robot as the source of truth.
        launch_args = [
            "ros2",
            "launch",
            "mavros",
            "apm.launch",
            f"fcu_url:={fcu_url}",
        ]
        launch_command = " ".join(shlex.quote(part) for part in launch_args)
        # Ubuntu real-robot package launch, kept as a comment on purpose:
        #   colcon build --base-paths kmu26_auv --symlink-install
        #   source rospkg/install/setup.bash
        #   ros2 launch hit25_auv_ros2 rov_start.launch.py fcu_url:=/dev/ttyACM0:57600
        # That path expects real hardware dependencies such as dvl_msgs,
        # robot_localization, DroneCAN battery bridge, and the DVL launch package.
        command = launch_command
        self._start_logged_ros_process(
            cmd=ros_bash_command(command, include_workspace=False),
            label="mavros",
            log_prefix="gui_mavros_pkg",
            attr_name="_ros_pkg_process",
            status_callback=self._set_ros_pkg_status,
        )
        if self._sim_stack_running() and self._ros_pkg_running():
            self._restart_sim_stack_after_mavros_mode_change(
                want_mavros_running=True,
                reason="MAVROS ON",
            )

    def _toggle_rviz(self) -> None:
        if self._rviz_running():
            self._terminate_process_group(self._rviz_process)
            self._set_rviz_status("rviz: stopping")
            self._refresh_ros2_buttons()
            return
        self._start_rviz()

    def _start_rviz(self) -> None:
        if not ROS_PACKAGE_RVIZ_CONFIG.exists():
            self._set_rviz_status(f"rviz config missing: {ROS_PACKAGE_RVIZ_CONFIG}")
            return
        try:
            rviz_config = prepare_ros2_rviz_config()
        except Exception as exc:
            self._set_rviz_status(f"rviz config prepare failed: {exc}")
            return
        command = " ".join(
            shlex.quote(part)
            for part in ("rviz2", "-d", str(rviz_config))
        )
        self._start_logged_ros_process(
            cmd=ros_bash_command(command, include_workspace=False),
            label="rviz",
            log_prefix="gui_rviz",
            attr_name="_rviz_process",
            status_callback=self._set_rviz_status,
        )

    def _sim_stack_running(self) -> bool:
        return self._sim_stack_process is not None and self._sim_stack_process.poll() is None

    def _set_sim_stack_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.sim_stack_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.sim_stack_status_var.set(text))
        except Exception:
            pass

    def _toggle_ping360_window(self) -> None:
        if self.ping360_window is not None and self.ping360_window.winfo_exists():
            self._close_ping360_window()
            return
        self._show_ping360_window()

    def _show_ping360_window(self) -> None:
        if self.ping360_window is not None and self.ping360_window.winfo_exists():
            self.ping360_window.deiconify()
            self.ping360_window.lift()
            return

        win = tk.Toplevel(self.root)
        win.title("Ping360 Control")
        win.geometry("520x300")
        win.minsize(460, 260)
        win.protocol("WM_DELETE_WINDOW", self._close_ping360_window)
        self.ping360_window = win

        outer = ttk.Frame(win, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)

        status_box = ttk.LabelFrame(outer, text="Status", padding=INNER_PADDING)
        status_box.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        status_box.columnconfigure(0, weight=1)
        ttk.Label(status_box, textvariable=self.ping360_view_status_var, anchor="w").grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Label(status_box, textvariable=self.ping360_summary_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=(2, 0)
        )

        view_box = ttk.LabelFrame(outer, text="Viewer", padding=INNER_PADDING)
        view_box.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(view_box, text="Open RViz/rqt", style="Info.TButton", command=self._start_ping360_view).pack(
            side=tk.LEFT
        )
        ttk.Button(view_box, text="Close Viewer", command=self._stop_ping360_view).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        params = ttk.LabelFrame(outer, text="Ping360 Params", padding=INNER_PADDING)
        params.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        for col in range(8):
            params.columnconfigure(col, weight=1 if col in (1, 3, 5, 7) else 0)

        ttk.Label(params, text="range").grid(row=0, column=0, sticky="w")
        ttk.Entry(params, width=6, textvariable=self.ping360_range_var).grid(
            row=0, column=1, sticky="ew", padx=(2, 6)
        )
        ttk.Label(params, text="step").grid(row=0, column=2, sticky="w")
        ttk.Entry(params, width=4, textvariable=self.ping360_num_steps_var).grid(
            row=0, column=3, sticky="ew", padx=(2, 6)
        )
        ttk.Label(params, text="gain").grid(row=0, column=4, sticky="w")
        ttk.Entry(params, width=4, textvariable=self.ping360_gain_var).grid(
            row=0, column=5, sticky="ew", padx=(2, 6)
        )
        ttk.Label(params, text="link").grid(row=0, column=6, sticky="w")
        ttk.Combobox(
            params,
            textvariable=self.ping360_interface_var,
            values=("ethernet", "usb", "rs485"),
            state="readonly",
            width=8,
        ).grid(row=0, column=7, sticky="ew", padx=(2, 0))

        ttk.Label(params, text="kHz").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(params, width=6, textvariable=self.ping360_frequency_var).grid(
            row=1, column=1, sticky="ew", padx=(2, 6), pady=(4, 0)
        )
        ttk.Label(params, text="start").grid(row=1, column=2, sticky="w", pady=(4, 0))
        ttk.Entry(params, width=5, textvariable=self.ping360_start_angle_var).grid(
            row=1, column=3, sticky="ew", padx=(2, 6), pady=(4, 0)
        )
        ttk.Label(params, text="stop").grid(row=1, column=4, sticky="w", pady=(4, 0))
        ttk.Entry(params, width=5, textvariable=self.ping360_stop_angle_var).grid(
            row=1, column=5, sticky="ew", padx=(2, 6), pady=(4, 0)
        )
        ttk.Label(params, text="grad").grid(row=1, column=6, sticky="w", pady=(4, 0))
        ttk.Button(params, text="Apply", style="Info.TButton", command=self._apply_ping360_params).grid(
            row=1, column=7, sticky="ew", padx=(2, 0), pady=(4, 0)
        )

        footer = ttk.Frame(outer)
        footer.grid(row=3, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        ttk.Label(
            footer,
            text="Publishes /ping360/config and opens the configured Ping360 view.",
            anchor="w",
            foreground="#64748b",
        ).grid(row=0, column=0, sticky="ew")
        ttk.Button(footer, text="Close", command=self._close_ping360_window).grid(row=0, column=1, padx=(8, 0))

    def _close_ping360_window(self) -> None:
        if self.ping360_window is not None and self.ping360_window.winfo_exists():
            self.ping360_window.destroy()
        self.ping360_window = None

    def _ping360_view_running(self) -> bool:
        return self._ping360_view_process is not None and self._ping360_view_process.poll() is None

    def _set_ping360_view_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.ping360_view_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.ping360_view_status_var.set(text))
        except Exception:
            pass

    def _start_ping360_view(self) -> None:
        if self._ping360_view_running():
            self._set_ping360_view_status("ping360 view: already open")
            return

        try:
            rviz_config = prepare_ping360_rviz_config()
        except Exception as exc:
            self._set_ping360_view_status(f"ping360 rviz config failed: {exc}")
            return

        command = "\n".join(
            [
                "if command -v rviz2 >/dev/null 2>&1; then",
                f"  exec rviz2 -d {shlex.quote(str(rviz_config))}",
                "elif command -v ros2 >/dev/null 2>&1; then",
                "  exec ros2 run rqt_image_view rqt_image_view /ping360/scan_image",
                "else",
                "  echo 'rviz2/ros2 not found after sourcing ROS setup' >&2",
                "  exit 127",
                "fi",
            ]
        )
        self._start_logged_ros_process(
            cmd=ros_bash_command(command, cwd=APP_ROOT, include_workspace=True),
            label="ping360 view",
            log_prefix="ping360_view",
            attr_name="_ping360_view_process",
            status_callback=self._set_ping360_view_status,
        )

    def _stop_ping360_view(self) -> None:
        proc = self._ping360_view_process
        if proc is None or proc.poll() is not None:
            self._set_ping360_view_status("ping360 view: closed")
            self._ping360_view_process = None
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        self._set_ping360_view_status("ping360 view: closing")
        self.node.push_event("ping360 view close requested")

    def _apply_ping360_params(self) -> None:
        range_m = self._read_float_var(self.ping360_range_var, 2.0, 0.75, 50.0)
        num_steps = self._read_int_var(self.ping360_num_steps_var, 1, 1, 10)
        gain = self._read_int_var(self.ping360_gain_var, 0, 0, 2)
        frequency_khz = self._read_int_var(self.ping360_frequency_var, 750, 500, 1000)
        start_grad = self._read_int_var(self.ping360_start_angle_var, 0, 0, 399)
        stop_grad = self._read_int_var(self.ping360_stop_angle_var, 399, 0, 399)
        interface_mode = self.ping360_interface_var.get().strip().lower()
        if interface_mode not in {"ethernet", "usb", "rs485"}:
            interface_mode = "ethernet"
            self.ping360_interface_var.set(interface_mode)

        self.node.publish_ping360_config(
            range_m=range_m,
            num_steps=num_steps,
            gain=gain,
            interface_mode=interface_mode,
            frequency_khz=frequency_khz,
            start_angle_grad=start_grad,
            stop_angle_grad=stop_grad,
        )
        self._set_ping360_view_status("ping360 config: published")

    def _start_sim_stack(self) -> None:
        if self._sim_stack_running():
            self._set_sim_stack_status("sim: already running")
            return
        if not START_SIM_STACK_SCRIPT.exists():
            self._set_sim_stack_status(f"sim script missing: {START_SIM_STACK_SCRIPT}")
            return
        if not os.access(START_SIM_STACK_SCRIPT, os.X_OK):
            self._set_sim_stack_status(f"sim script is not executable: {START_SIM_STACK_SCRIPT}")
            return

        if self._rc_replay_running():
            self._stop_rc_replay()
        self.rc_override_enabled.set(False)
        self.node.publish_rc_release()

        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        # GUI-started SITL runs should go through ArduSub, not also inject the
        # same RC override directly into MuJoCo.
        env["ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK"] = "0"
        log_dir = SIM_STACK_DIR / "logs"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"gui_start_stack_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            log_file = log_path.open("w", encoding="utf-8")
        except Exception as exc:
            self._set_sim_stack_status(f"sim log open failed: {exc}")
            return

        try:
            cmd = [str(START_SIM_STACK_SCRIPT)]
            if self._ros_pkg_running():
                cmd.append("--ros2-real-pkg-compat")
            proc = subprocess.Popen(
                cmd,
                cwd=str(SIM_STACK_DIR),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                start_new_session=True,
            )
        except Exception as exc:
            log_file.close()
            self._set_sim_stack_status(f"sim start failed: {exc}")
            return
        finally:
            try:
                log_file.close()
            except Exception:
                pass

        self._sim_stack_process = proc
        self._sim_stack_log_path = log_path
        self._set_sim_stack_status("sim: starting SITL/MuJoCo")
        self.node.push_event(f"sim stack start requested: {log_path.name}")
        self._sim_stack_thread = threading.Thread(
            target=self._watch_sim_stack_output,
            args=(proc, log_path),
            daemon=True,
        )
        self._sim_stack_thread.start()

    def _watch_sim_stack_output(self, proc: subprocess.Popen[str], log_path: Path) -> None:
        last_line = ""
        try:
            with log_path.open("r", encoding="utf-8", errors="replace") as log_stream:
                while True:
                    raw_line = log_stream.readline()
                    if raw_line:
                        line = raw_line.strip()
                        if not line:
                            continue
                        last_line = line
                        if line.startswith(
                            ("[start]", "[reset]", "[launch]", "[runtime]", "[physics]", "[sitl]", "[model]")
                        ):
                            short_line = line if len(line) <= 150 else f"{line[:147]}..."
                            self.node.push_event(short_line)
                            if line.startswith(("[start]", "[reset]")):
                                self._set_sim_stack_status(f"sim: {short_line}")
                        continue
                    if proc.poll() is not None:
                        break
                    time.sleep(0.1)
                for raw_line in log_stream:
                    line = raw_line.strip()
                    if line:
                        last_line = line
            rc = proc.returncode if proc.returncode is not None else proc.wait()
        except Exception as exc:
            rc = -1
            last_line = f"reader failed: {exc}"

        def finish() -> None:
            if self._sim_stack_process is proc:
                self._sim_stack_process = None
            if rc == 0:
                self.sim_stack_status_var.set("sim: exited")
                self.node.push_event("sim stack exited")
            elif rc < 0:
                self.sim_stack_status_var.set("sim: stopped")
                self.node.push_event("sim stack stopped")
            else:
                text = last_line if last_line else f"rc={rc}"
                self.sim_stack_status_var.set(f"sim failed: {text}")
                self.node.push_event(f"sim stack failed: {text}")

        try:
            self.root.after(0, finish)
        except Exception:
            pass

    def _terminate_sim_stack_process(self) -> None:
        proc = self._sim_stack_process
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass

    def _stop_sim_stack(self) -> None:
        self._terminate_sim_stack_process()
        self._set_sim_stack_status("sim: resetting stack")
        self.node.push_event("sim stack reset requested")
        if self._sim_stack_reset_thread is not None and self._sim_stack_reset_thread.is_alive():
            return
        self._sim_stack_reset_thread = threading.Thread(target=self._run_sim_stack_reset, daemon=True)
        self._sim_stack_reset_thread.start()

    def _run_sim_stack_reset(self) -> None:
        if not RESET_SIM_STACK_SCRIPT.exists():
            self._set_sim_stack_status(f"reset script missing: {RESET_SIM_STACK_SCRIPT}")
            return
        cmd = [str(RESET_SIM_STACK_SCRIPT), "--wipe-eeprom"]
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(SIM_STACK_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            assert proc.stdout is not None
            for raw_line in proc.stdout:
                line = raw_line.strip()
                if not line:
                    continue
                if line.startswith("[reset]"):
                    short_line = line if len(line) <= 150 else f"{line[:147]}..."
                    self.node.push_event(short_line)
                    self._set_sim_stack_status(f"sim: {short_line}")
            rc = proc.wait()
        except Exception as exc:
            self._set_sim_stack_status(f"sim reset failed: {exc}")
            self.node.push_event(f"sim reset failed: {exc}")
            return
        if rc == 0:
            self._set_sim_stack_status("sim: stopped/reset")
            self.node.push_event("sim stack stopped/reset")
        else:
            self._set_sim_stack_status(f"sim reset failed: rc={rc}")
            self.node.push_event(f"sim reset failed: rc={rc}")
