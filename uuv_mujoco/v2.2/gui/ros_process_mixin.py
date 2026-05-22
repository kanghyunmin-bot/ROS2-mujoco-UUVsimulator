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
    @staticmethod
    def _env_flag(name: str, default: bool = False) -> bool:
        raw = os.environ.get(name)
        if raw is None:
            return bool(default)
        return raw.strip().lower() in {"1", "true", "yes", "on"}

    @staticmethod
    def _arg_present(args: list[str], option: str) -> bool:
        return any(arg == option or arg.startswith(f"{option}=") for arg in args)

    @staticmethod
    def _any_arg_present(args: list[str], options: tuple[str, ...]) -> bool:
        return any(RosProcessMixin._arg_present(args, option) for option in options)

    @staticmethod
    def _default_sim_stack_backend() -> str:
        return "local"

    def _sim_stack_backend(self) -> str:
        backend = os.environ.get("UUV_SITL_BACKEND", self._default_sim_stack_backend())
        backend = backend.strip().lower()
        return backend or self._default_sim_stack_backend()

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

    def _gui_external_mavros_controls_enabled(self) -> bool:
        return self._ros_pkg_running() and self._env_flag("UUV_GUI_USE_EXTERNAL_MAVROS", False)

    def _gui_sim_stack_env(self) -> dict[str, str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"

        forced = {
            # GUI-started SITL must keep RC override on the ArduSub closed-loop
            # path. Direct MuJoCo fallback is only an explicit debug override.
            "ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK": "0",
            "ROS2_UUV_MAVROS_RC_PWM_SPAN": "400",
            # Real/QGC/rosbag RC values stay on the 1500-centered convention.
            # ArduSub 4.1.2 with RC3_MIN/MAX=1100/1900 interprets RC3=1500 as
            # neutral in ALT_HOLD; remapping it corrupts the vertical target.
            # ArduPilot's JSON backend sends the real-time servo stream. MAVLink
            # SERVO_OUTPUT_RAW is telemetry-rate and too delayed for plant control.
            "ROS2_UUV_SITL_JSON_SERVO_FALLBACK": "1",
            # Do not synthesize RC in the simulator. GUI/QGC/rosbag input must be
            # the only RC source so ALT_HOLD target changes match the real path.
            "ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE": "0",
            # GUI Start should only boot the plant. Auto arm + ALT_HOLD can latch
            # stale QGC/RC state and move before the operator commands it.
            "SITL_AUTO_SAFE_SEQUENCE": "0",
            # Match the real robot estimator path: EKF3 with ExternalNav
            # for XY/velocity/yaw and Bar30 for vertical position.
            "SITL_USE_REAL_PARAM_FILE": "1",
            "SITL_EKF3_EXTNAV": "1",
            "SITL_AHRS_EKF_TYPE": "3",
            "ROS2_UUV_SITL_EXTNAV_ENABLE": "1",
            # Explicit GUI/QGC/test arm/mode commands must still reach ArduSub.
            "ROS2_UUV_MAVROS_FORWARD_ARM_MODE": "1",
            # The bridge fixes SITL vertical feedback to Bar30 depth and NED
            # down-positive velocity for JSON and ExternalNav.
            # Match the real robot: ALT_HOLD vertical sensing comes from Bar30,
            # not the SITL rangefinder backend.
            "SITL_RNGFND1_TYPE": "0",
            "SITL_SURFACE_DEPTH": "-10.0",
            "SITL_SURFACE_MAX_THR": "0.1",
            # GUI arm/mode buttons wait for fresh Bar30/ExternalNav/IMU streams
            # plus a short estimator settle. The long boot guard made startup
            # look broken on slower machines without improving the contract.
            "UUV_GUI_REQUIRE_ARM_MODE_EKF_SETTLE": "1",
            "UUV_GUI_ARM_MODE_EKF_SETTLE_S": "3.0",
        }
        env.update(forced)

        defaults = {
            "UUV_RUNTIME_PROFILE": "balanced",
            "SITL_SENSOR_HZ_DEFAULT": "60",
            "SITL_THRUSTER_LOOP_HZ_DEFAULT": "80",
            "UUV_MUJOCO_VIEWER_FPS": "60",
            "ROS2_UUV_SPIN_HZ": "100",
            "ROS2_UUV_DEMAND_PROBE_PERIOD_S": "1.0",
            "ROS2_UUV_SITL_CMD_DEBUG": "0",
            "ROS2_UUV_SITL_MAVLINK_POLL_HZ": "50",
            "ROS2_UUV_SITL_COMMAND_POLL_HZ": "25",
            "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT": "same",
            "ROS2_UUV_ARM_MODE_BOOT_GUARD_S": "4",
        }
        profile = env.get("UUV_RUNTIME_PROFILE", "balanced").strip().lower()
        if profile == "low":
            defaults.update(
                {
                    "SITL_SENSOR_HZ_DEFAULT": "30",
                    "SITL_THRUSTER_LOOP_HZ_DEFAULT": "60",
                    "UUV_MUJOCO_VIEWER_FPS": "30",
                    "ROS2_UUV_SPIN_HZ": "60",
                }
            )
        elif profile == "high":
            defaults.update(
                {
                    "SITL_SENSOR_HZ_DEFAULT": "120",
                    "SITL_THRUSTER_LOOP_HZ_DEFAULT": "100",
                    "UUV_MUJOCO_VIEWER_FPS": "60",
                    "ROS2_UUV_SPIN_HZ": "150",
                }
            )
        for key, value in defaults.items():
            env.setdefault(key, value)
        return env

    def _append_mavros_surface_args(self, cmd: list[str]) -> None:
        if self._gui_external_mavros_controls_enabled():
            cmd.append("--ros2-real-pkg-compat")
            self.node.push_event("MAVROS surface: external node owns arm/mode/RC")
            return
        self.node.push_event("MAVROS surface: internal sim bridge owns arm/mode/RC")

    def _normalized_sim_extra_args(self, extra_args: list[str] | None) -> list[str]:
        wrapper_only_args = {
            "--direct-mavlink",
            "--with-qgc-stop",
            "--no-wait-ready",
            "--sitl-no-rebuild",
            "--sitl-rebuild",
            "--sitl-no-display",
            "--no-ekf-stable",
            "--param-tune",
            "--wipe-eeprom",
            "--keep-eeprom",
            "--no-reset",
        }
        args: list[str] = []
        dropped: list[str] = []
        for arg in extra_args or []:
            option = arg.split("=", 1)[0]
            if option in wrapper_only_args:
                dropped.append(arg)
                continue
            args.append(arg)
        if dropped:
            self.node.push_event(
                "sim launch ignored wrapper-only args: " + " ".join(dropped)
            )
        display_available = (
            sys.platform == "darwin"
            or bool(os.environ.get("DISPLAY"))
            or bool(os.environ.get("WAYLAND_DISPLAY"))
        )
        viewer_enabled = self._env_flag("UUV_GUI_MUJOCO_VIEWER", display_available)
        if not viewer_enabled and not self._arg_present(args, "--headless"):
            args.append("--headless")
            self.node.push_event("sim viewer: headless MuJoCo runtime")
        elif viewer_enabled:
            self.node.push_event("sim viewer: MuJoCo GLFW viewer enabled")
        if not self._any_arg_present(args, ("--qgc-video", "--no-qgc-video")):
            args.append("--no-qgc-video")
        return args

    def _append_initial_depth_args(self, cmd: list[str], launch_extra_args: list[str]) -> None:
        if self._arg_present(launch_extra_args, "--initial-depth-m"):
            return

        # Set the initial pose before the first JSON sample. Do not pin the
        # vehicle by default: the artificial hold/release path hides startup
        # physics and can inject an AltHold transient that the real vehicle
        # does not have.
        gui_initial_depth_m = os.environ.get("UUV_GUI_INITIAL_DEPTH_M", "0.2501").strip()
        if not gui_initial_depth_m:
            return

        has_hold_target = self._arg_present(launch_extra_args, "--initial-depth-hold-target-m")
        has_hold_flag = self._arg_present(launch_extra_args, "--hold-initial-depth-until-release")
        hold_until_release = self._env_flag("UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE", False)
        hold_target_m = os.environ.get(
            "UUV_GUI_INITIAL_DEPTH_HOLD_TARGET_M",
            gui_initial_depth_m,
        ).strip() if hold_until_release else ""

        cmd.extend(["--initial-depth-m", gui_initial_depth_m])
        if hold_until_release and hold_target_m and not has_hold_target:
            cmd.extend(["--initial-depth-hold-target-m", hold_target_m])
        if hold_until_release and not has_hold_flag:
            cmd.append("--hold-initial-depth-until-release")
        self.node.push_event(f"sim initial depth: base_link={gui_initial_depth_m} m")

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
    def _matching_process_commands(patterns: tuple[str, ...]) -> list[str]:
        try:
            result = subprocess.run(
                ["ps", "-axo", "pid=,command="],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=1.0,
                check=False,
            )
        except Exception:
            return []
        if result.returncode != 0:
            return []

        compiled = [re.compile(pattern) for pattern in patterns]
        matches: list[str] = []
        own_pid = os.getpid()
        for raw_line in result.stdout.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            try:
                pid_text, command = line.split(maxsplit=1)
                pid = int(pid_text)
            except ValueError:
                continue
            if pid == own_pid:
                continue
            if any(regex.search(command) for regex in compiled):
                matches.append(command)
        return matches

    @staticmethod
    def _terminate_process_group(proc: subprocess.Popen[str] | None, timeout_s: float = 4.0) -> None:
        if proc is None or proc.poll() is not None:
            return
        pgid: int | None = None
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        try:
            proc.wait(timeout=max(0.2, float(timeout_s)))
            return
        except Exception:
            pass
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGKILL)
            else:
                proc.kill()
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        try:
            proc.wait(timeout=1.0)
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
        if not self._tracked_sim_stack_running() and self._external_sim_stack_running():
            self._external_sim_stack_running_cached = True
            self._set_sim_stack_status(f"sim: external stack running; Stop/Reset before {reason}")
            self.node.push_event(f"sim restart skipped for {reason}: external stack is not GUI-owned")
            self._refresh_sim_stack_controls()
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
        # Simulation path: this starts an external MAVROS helper for inspection
        # and RViz workflows.  The simulator keeps its own lightweight MAVROS
        # control surface by default because it is the deterministic closed-loop
        # path on macOS+Docker.  Set UUV_GUI_USE_EXTERNAL_MAVROS=1 only when
        # the external MAVROS node should own /mavros arm/mode/RC services.
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

    def _tracked_sim_stack_running(self) -> bool:
        return self._sim_stack_process is not None and self._sim_stack_process.poll() is None

    def _external_sim_stack_commands(self) -> list[str]:
        return self._matching_process_commands(
            (
                r"(^|[/ ]|\./)start_docker_sitl_mujoco_mj311\.sh($| )",
                r"(^|[/ ]|\./)start_sitl_mujoco_mj311\.sh($| )",
                r"(^|[/ ]|\./)launch_uuv_sim\.sh .*--sitl",
                r"(^|[/ ]|\./)run_urdf_full\.py .*--sitl",
            )
        )

    def _external_sim_stack_running(self) -> bool:
        return bool(self._external_sim_stack_commands())

    def _sim_stack_running(self) -> bool:
        return self._tracked_sim_stack_running() or self._external_sim_stack_running()

    def _wait_for_external_sim_stack_exit(self, timeout_s: float = 10.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            if not self._external_sim_stack_running():
                self._external_sim_stack_running_cached = False
                return True
            time.sleep(0.2)
        self._external_sim_stack_running_cached = self._external_sim_stack_running()
        return not self._external_sim_stack_running_cached

    def _refresh_sim_stack_controls(self) -> None:
        reset_running = (
            self._sim_stack_reset_thread is not None
            and self._sim_stack_reset_thread.is_alive()
        )
        running = (
            self._tracked_sim_stack_running()
            or bool(getattr(self, "_external_sim_stack_running_cached", False))
            or reset_running
        )
        if getattr(self, "sim_stack_start_button", None) is not None:
            self.sim_stack_start_button.config(
                text="Stack Running" if running else "Start SITL/MuJoCo",
                state=tk.DISABLED if running else tk.NORMAL,
            )
        if getattr(self, "sim_stack_stop_button", None) is not None:
            self.sim_stack_stop_button.config(state=tk.NORMAL)

    def _refresh_sim_stack_status(self) -> None:
        now = time.monotonic()
        last_probe = getattr(self, "_last_sim_stack_probe_wall", -1.0)
        if last_probe >= 0.0 and now - last_probe < 3.0:
            self._refresh_sim_stack_controls()
            return
        self._last_sim_stack_probe_wall = now
        reset_running = (
            self._sim_stack_reset_thread is not None
            and self._sim_stack_reset_thread.is_alive()
        )
        if reset_running:
            self._refresh_sim_stack_controls()
            return
        tracked = self._tracked_sim_stack_running()
        external = self._external_sim_stack_running()
        self._external_sim_stack_running_cached = external
        current = self.sim_stack_status_var.get()
        if tracked:
            if current.startswith(("sim: stopped", "sim: exited", "sim failed")):
                self._set_sim_stack_status("sim: running")
        elif external:
            if not current.startswith("sim: running (external)"):
                self._set_sim_stack_status("sim: running (external)")
        elif current.startswith(("sim: running", "sim: backend running")):
            self._set_sim_stack_status("sim: stopped")
        self._refresh_sim_stack_controls()

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
        win.geometry("520x350")
        win.minsize(460, 320)
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

        power_box = ttk.LabelFrame(outer, text="Sonar", padding=INNER_PADDING)
        power_box.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        ttk.Checkbutton(
            power_box,
            text="Enabled",
            variable=self.ping360_enabled_var,
            command=self._toggle_ping360_enabled,
        ).pack(side=tk.LEFT)
        ttk.Button(power_box, text="ON", style="Info.TButton", command=lambda: self._set_ping360_enabled(True)).pack(
            side=tk.LEFT, padx=(10, 0)
        )
        ttk.Button(power_box, text="OFF", style="Danger.TButton", command=lambda: self._set_ping360_enabled(False)).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        view_box = ttk.LabelFrame(outer, text="Viewer", padding=INNER_PADDING)
        view_box.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(view_box, text="Open RViz/rqt", style="Info.TButton", command=self._start_ping360_view).pack(
            side=tk.LEFT
        )
        ttk.Button(view_box, text="Close Viewer", command=self._stop_ping360_view).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        params = ttk.LabelFrame(outer, text="Ping360 Params", padding=INNER_PADDING)
        params.grid(row=3, column=0, sticky="ew", pady=(0, 6))
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
        footer.grid(row=4, column=0, sticky="ew")
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

    def _set_ping360_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        self.ping360_enabled_var.set(enabled)
        self.node.publish_ping360_enabled(enabled)
        self._set_ping360_view_status(f"ping360 sonar: {'on' if enabled else 'off'} requested")

    def _toggle_ping360_enabled(self) -> None:
        self._set_ping360_enabled(bool(self.ping360_enabled_var.get()))

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

    def _start_sim_stack(self, extra_args: list[str] | None = None) -> None:
        if self._tracked_sim_stack_running():
            self._set_sim_stack_status("sim: already running")
            return
        if self._external_sim_stack_running():
            self._external_sim_stack_running_cached = True
            self._set_sim_stack_status("sim: already running (external); Stop/Reset first")
            self._refresh_sim_stack_controls()
            return
        backend = self._sim_stack_backend()
        start_script = START_DOCKER_SIM_STACK_SCRIPT if backend == "docker" else START_SIM_STACK_SCRIPT
        if not start_script.exists():
            self._set_sim_stack_status(f"sim script missing: {start_script}")
            return
        if not os.access(start_script, os.X_OK):
            self._set_sim_stack_status(f"sim script is not executable: {start_script}")
            return

        if self._rc_replay_running():
            self._stop_rc_replay()
        self.rc_override_enabled.set(False)
        self.node.publish_rc_release()

        env = self._gui_sim_stack_env()
        log_dir = SIM_STACK_DIR / "logs"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"gui_start_stack_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            log_file = log_path.open("w", encoding="utf-8")
        except Exception as exc:
            self._set_sim_stack_status(f"sim log open failed: {exc}")
            return

        try:
            cmd = [str(start_script)]
            self._append_mavros_surface_args(cmd)
            if (
                backend != "docker"
                and self._env_flag("UUV_GUI_SITL_DIRECT_MAVLINK", False)
                and not self._arg_present(extra_args or [], "--direct-mavlink")
            ):
                cmd.append("--direct-mavlink")
                self.node.push_event("SITL transport: direct MAVLink outputs")
            if (
                backend != "docker"
                and not self._env_flag("UUV_GUI_SITL_REBUILD", False)
            ):
                cmd.append("--sitl-no-rebuild")
                self.node.push_event("SITL rebuild skipped: using existing ArduSub binary")
            launch_extra_args = self._normalized_sim_extra_args(extra_args)
            self._append_initial_depth_args(cmd, launch_extra_args)

            if launch_extra_args:
                cmd.extend(launch_extra_args)
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
        self._external_sim_stack_running_cached = False
        self._sim_stack_owned_by_gui = True
        start_label = "Docker SITL/MuJoCo" if backend == "docker" else "SITL/MuJoCo"
        self._set_sim_stack_status(f"sim: starting {start_label}")
        self._refresh_sim_stack_controls()
        self.node.push_event(f"sim stack start requested ({backend}): {log_path.name}")
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
                            (
                                "[start]",
                                "[docker-start]",
                                "[docker-sitl]",
                                "[reset]",
                                "[launch]",
                                "[runtime]",
                                "[physics]",
                                "[sitl]",
                                "[model]",
                            )
                        ):
                            short_line = line if len(line) <= 150 else f"{line[:147]}..."
                            self.node.push_event(short_line)
                            if line.startswith(("[start]", "[docker-start]", "[reset]")):
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
                self._sim_stack_owned_by_gui = False
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
            self._refresh_sim_stack_controls()

        try:
            self.root.after(0, finish)
        except Exception:
            pass

    def _terminate_sim_stack_process(self) -> None:
        proc = self._sim_stack_process
        if proc is None or proc.poll() is not None:
            return
        self._terminate_process_group(proc, timeout_s=5.0)
        if self._sim_stack_process is proc:
            self._sim_stack_process = None
            self._sim_stack_owned_by_gui = False

    def _reset_sim_stack_blocking(self, timeout_s: float = 12.0) -> None:
        if self._sim_stack_backend() == "docker":
            self._stop_docker_sitl_blocking(timeout_s=timeout_s)
        if not RESET_SIM_STACK_SCRIPT.exists():
            return
        try:
            subprocess.run(
                [str(RESET_SIM_STACK_SCRIPT), "--wipe-eeprom"],
                cwd=str(SIM_STACK_DIR),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=float(timeout_s),
                check=False,
                start_new_session=True,
            )
        except Exception:
            pass

    def _stop_docker_sitl_blocking(self, timeout_s: float = 12.0) -> None:
        if not STOP_DOCKER_SITL_SCRIPT.exists():
            return
        try:
            subprocess.run(
                [str(STOP_DOCKER_SITL_SCRIPT)],
                cwd=str(SIM_STACK_DIR),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=float(timeout_s),
                check=False,
                start_new_session=True,
            )
        except Exception:
            pass

    def _stop_sim_stack(self) -> None:
        self._terminate_sim_stack_process()
        self._set_sim_stack_status("sim: resetting stack")
        self._refresh_sim_stack_controls()
        self.node.push_event("sim stack reset requested")
        if self._sim_stack_reset_thread is not None and self._sim_stack_reset_thread.is_alive():
            return
        self._sim_stack_reset_thread = threading.Thread(target=self._run_sim_stack_reset, daemon=True)
        self._sim_stack_reset_thread.start()

    def _run_sim_stack_reset(self) -> None:
        if not RESET_SIM_STACK_SCRIPT.exists():
            self._set_sim_stack_status(f"reset script missing: {RESET_SIM_STACK_SCRIPT}")
            return
        if self._sim_stack_backend() == "docker":
            self.node.push_event("docker SITL stop requested")
            self._stop_docker_sitl_blocking(timeout_s=20.0)
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
            if self._wait_for_external_sim_stack_exit(timeout_s=3.0):
                self._sim_stack_owned_by_gui = False
                self._set_sim_stack_status("sim: stopped/reset")
                self.node.push_event("sim stack stopped/reset")
            else:
                self._set_sim_stack_status("sim: reset done; external process still running")
                self.node.push_event("sim reset done; external process still running")
        else:
            self._set_sim_stack_status(f"sim reset failed: rc={rc}")
            self.node.push_event(f"sim reset failed: rc={rc}")
        try:
            self.root.after(0, self._refresh_sim_stack_controls)
        except Exception:
            pass
