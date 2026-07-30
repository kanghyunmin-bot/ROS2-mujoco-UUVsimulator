import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Iterable


def subprocess_env() -> dict[str, str]:
    env = os.environ.copy()
    python_bin_dir = str(Path(sys.executable).resolve().parent)
    path_entries = [
        entry
        for entry in env.get("PATH", "").split(os.pathsep)
        if entry and Path(entry).resolve() != Path(python_bin_dir)
    ]
    env["PATH"] = os.pathsep.join([python_bin_dir, *path_entries])
    env.setdefault("PYTHONUNBUFFERED", "1")
    env.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
    return env


def _default_dronecan_python() -> str:
    candidate = os.path.expanduser("~/miniconda3/envs/auv_ros2/bin/python")
    if os.path.exists(candidate):
        return candidate
    return sys.executable


def _default_bag_output_root() -> Path:
    home_catkin_ws = Path.home() / "catkin_ws"
    if home_catkin_ws.exists():
        return home_catkin_ws

    cwd = Path.cwd()
    if (cwd / "src").is_dir() and (cwd / "install").is_dir():
        return cwd

    return home_catkin_ws


class ManagedProcess:
    def __init__(self, name: str, cmd: list[str], log_buffer: deque[str]):
        self.name = name
        self.cmd = cmd
        self.log_buffer = log_buffer
        self.process: subprocess.Popen[str] | None = None
        self.pgid: int | None = None
        self._reader_thread: threading.Thread | None = None

    def start(self) -> None:
        if self.is_running:
            raise RuntimeError(f"{self.name} is already running")

        self._log(f"$ {' '.join(self.cmd)}")
        self.process = subprocess.Popen(
            self.cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=subprocess_env(),
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.pgid = os.getpgid(self.process.pid)
        self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
        self._reader_thread.start()

    def stop(self) -> None:
        if self.pgid is None and (
            self.process is None or self.process.poll() is not None
        ):
            return

        self._log(f"[{self.name}] stopping")
        pgid = self.pgid
        if pgid is None and self.process is not None:
            try:
                pgid = os.getpgid(self.process.pid)
            except ProcessLookupError:
                return

        try:
            # ROS launch nodes use SIGINT for an orderly shutdown. In
            # particular, the pinger controller publishes RC channel release
            # values before its launch group exits.
            _kill_process_group(pgid, signal.SIGINT)
            if self.process is not None and self.process.poll() is None:
                self.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._log(f"[{self.name}] graceful stop timed out; terminating")
            _kill_process_group(pgid, signal.SIGTERM)
            try:
                if self.process is not None:
                    self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._log(f"[{self.name}] force killing")
                _kill_process_group(pgid, signal.SIGKILL)
                if self.process is not None:
                    self.process.wait(timeout=2.0)
                self.pgid = None
                return

        if not _wait_for_process_group_exit(pgid, timeout=2.0):
            self._log(f"[{self.name}] force killing remaining process group {pgid}")
            _kill_process_group(pgid, signal.SIGKILL)
            _wait_for_process_group_exit(pgid, timeout=2.0)
        self.pgid = None

    @property
    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def _read_output(self) -> None:
        assert self.process is not None
        if self.process.stdout is None:
            return
        for line in self.process.stdout:
            self._log(f"[{self.name}] {line.rstrip()}")
        return_code = self.process.wait()
        self._log(f"[{self.name}] exited with code {return_code}")

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        entry = f"{timestamp} {message}"
        self.log_buffer.append(entry)
        print(entry, file=sys.stdout, flush=True)


class ProcessManager:
    def __init__(
        self,
        robot_package: str = "auv",
        robot_launch: str = "localization_test.launch.py",
        start_dronecan_allocator: bool = True,
        dronecan_can_interface: str = "can1",
        dronecan_allocator_node_id: int = 126,
        dronecan_allocator_db: str = "",
        dronecan_python: str = "",
        pinger_package: str = "auv_pinger_homing",
        pinger_launch: str = "pinger_homing_real.launch.py",
        realsense_package: str = "realsense2_camera",
        realsense_launch: str = "rs_launch.py",
        audio_capture_package: str = "audio_capture",
        audio_capture_launch: str = "capture.launch.py",
    ):
        self.robot_package = robot_package
        self.robot_launch = robot_launch
        self.start_dronecan_allocator_on_startup = start_dronecan_allocator
        self.dronecan_can_interface = dronecan_can_interface
        self.dronecan_allocator_node_id = dronecan_allocator_node_id
        self.dronecan_allocator_db = dronecan_allocator_db
        self.dronecan_python = dronecan_python or _default_dronecan_python()
        self.pinger_package = pinger_package
        self.pinger_launch = pinger_launch
        self.realsense_package = realsense_package
        self.realsense_launch = realsense_launch
        self.audio_capture_package = audio_capture_package
        self.audio_capture_launch = audio_capture_launch
        self.logs: deque[str] = deque(maxlen=500)
        self._dronecan_allocator: ManagedProcess | None = None
        self._stack: ManagedProcess | None = None
        self._realsense_camera: ManagedProcess | None = None
        self._audio_capture: ManagedProcess | None = None
        self._vision_yolo: ManagedProcess | None = None
        self._vision_mission: ManagedProcess | None = None
        self._pinger: ManagedProcess | None = None
        self._bag: ManagedProcess | None = None
        self._bag_output: str = ""

    def start_dronecan_allocator(self) -> None:
        if not self.start_dronecan_allocator_on_startup:
            return
        if self.dronecan_allocator_running:
            return

        cmd = [
            self.dronecan_python,
            "-m",
            "auv_web_gui.dronecan_dynamic_id_allocator",
            "--can-interface",
            self.dronecan_can_interface,
            "--node-id",
            str(self.dronecan_allocator_node_id),
        ]
        if self.dronecan_allocator_db:
            cmd.extend(["--database-storage", self.dronecan_allocator_db])
        self._dronecan_allocator = ManagedProcess("dronecan_allocator", cmd, self.logs)
        self._dronecan_allocator.start()

    def start_stack(self, launch_args: dict[str, str] | None = None) -> None:
        if self.stack_running:
            raise RuntimeError("robot stack is already running")

        args = dict(launch_args or {})
        if self._uses_external_dronecan_allocator():
            args.setdefault("enable_battery_dynamic_id_server", "false")

        cmd = ["ros2", "launch", self.robot_package, self.robot_launch]
        for key, value in args.items():
            if value != "":
                cmd.append(f"{key}:={value}")

        self._stack = ManagedProcess("localization_test", cmd, self.logs)
        self._realsense_camera = ManagedProcess(
            "realsense_camera",
            _ros2_launch_command(
                self.realsense_package,
                self.realsense_launch,
                None,
            ),
            self.logs,
        )
        self._audio_capture = ManagedProcess(
            "audio_capture",
            _ros2_launch_command(
                self.audio_capture_package,
                self.audio_capture_launch,
                None,
            ),
            self.logs,
        )

        started = []
        try:
            for process in (
                self._stack,
                self._realsense_camera,
                self._audio_capture,
            ):
                process.start()
                started.append(process)
        except Exception as exc:
            for process in reversed(started):
                process.stop()
            raise RuntimeError(f"failed to start robot stack: {exc}") from exc

    def start_pinger(self, launch_args: dict[str, str] | None = None) -> None:
        if self._pinger and self._pinger.is_running:
            raise RuntimeError("pinger homing is already running")
        args = {
            "dry_run": "true",
            "use_audio_capture": "false",
            "use_hydrophone_estimator": "true",
            "use_rc_mux": "true",
        }
        args.update(launch_args or {})
        cmd = ["ros2", "launch", self.pinger_package, self.pinger_launch]
        for key, value in args.items():
            if value != "":
                cmd.append(f"{key}:={value}")
        self._pinger = ManagedProcess("pinger_homing", cmd, self.logs)
        self._pinger.start()

    def stop_pinger(self) -> None:
        if self._pinger:
            self._pinger.stop()

    def stop_stack(self) -> None:
        if self._audio_capture:
            self._audio_capture.stop()
        if self._realsense_camera:
            self._realsense_camera.stop()
        if self._stack:
            self._stack.stop()
        self._stop_orphaned_stack_groups()

    def start_vision_yolo(self, launch_args: dict[str, str] | None = None) -> None:
        if self.vision_yolo_running:
            raise RuntimeError("vision YOLO detector is already running")
        self._vision_yolo = ManagedProcess(
            "vision_yolo",
            _ros2_launch_command(
                "auv_buoy_vision_control",
                "laptop_yolo_detection.launch.py",
                launch_args,
            ),
            self.logs,
        )
        self._vision_yolo.start()

    def stop_vision_yolo(self) -> None:
        if self._vision_yolo:
            self._vision_yolo.stop()

    def start_vision_mission(self, launch_args: dict[str, str] | None = None) -> None:
        if self.vision_mission_running:
            raise RuntimeError("vision mission controller is already running")
        self._vision_mission = ManagedProcess(
            "vision_mission",
            _ros2_launch_command(
                "auv_buoy_vision_control",
                "auv_bbox_controller.launch.py",
                launch_args,
            ),
            self.logs,
        )
        self._vision_mission.start()

    def stop_vision_mission(self) -> None:
        if self._vision_mission:
            self._vision_mission.stop()

    def stop_vision(self) -> None:
        self.stop_vision_mission()
        self.stop_vision_yolo()

    def restart_localization_filter(self) -> dict:
        groups = self._stack_process_groups()
        if not groups:
            raise RuntimeError("robot stack is not running")

        old_pids = _matching_process_pids(
            groups,
            executable="ekf_node",
            marker="robot_localization",
        )
        if not old_pids:
            raise RuntimeError("robot_localization ekf_node is not running")

        self._log(
            "[localization_test] restarting robot_localization ekf_node "
            f"(pid {', '.join(str(pid) for pid in old_pids)})"
        )
        for pid in old_pids:
            _signal_process(pid, signal.SIGTERM)

        if not _wait_for_pids_to_exit(
            old_pids,
            executable="ekf_node",
            marker="robot_localization",
            timeout=3.0,
        ):
            self._log("[localization_test] force killing ekf_node")
            for pid in old_pids:
                _signal_process(pid, signal.SIGKILL)
            _wait_for_pids_to_exit(
                old_pids,
                executable="ekf_node",
                marker="robot_localization",
                timeout=2.0,
            )

        new_pids = _wait_for_new_matching_pids(
            groups,
            old_pids,
            executable="ekf_node",
            marker="robot_localization",
            timeout=5.0,
        )
        if not new_pids:
            raise RuntimeError("robot_localization ekf_node did not respawn")

        self._log(
            "[localization_test] robot_localization ekf_node respawned "
            f"(pid {', '.join(str(pid) for pid in new_pids)})"
        )
        return {"old_pids": old_pids, "new_pids": new_pids}

    def start_bag(
        self,
        topics: Iterable[str] | None = None,
        output_root: str | os.PathLike[str] | None = None,
        record_all: bool = False,
    ) -> str:
        if self._bag and self._bag.is_running:
            raise RuntimeError("bag recording is already running")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        root = Path(output_root).expanduser() if output_root else _default_bag_output_root()
        output_dir = root / f"localization_{timestamp}"
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        topic_list = [topic for topic in (topics or []) if topic]
        if record_all:
            cmd = ["ros2", "bag", "record", "-o", str(output_dir), "-a"]
        else:
            if not topic_list:
                raise RuntimeError("at least one topic must be selected")
            cmd = ["ros2", "bag", "record", "-o", str(output_dir), *topic_list]
        self._bag = ManagedProcess("bag", cmd, self.logs)
        self._bag_output = str(output_dir)
        self._bag.start()
        return str(output_dir)

    def list_topics(self) -> list[str]:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=subprocess_env(),
            text=True,
            timeout=3.0,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or "failed to list ROS topics")
        return sorted(line.strip() for line in result.stdout.splitlines() if line.strip())

    def stop_bag(self) -> None:
        if self._bag:
            self._bag.stop()

    def status(self) -> dict:
        (
            robot_launch_running,
            realsense_camera_running,
            audio_capture_running,
        ) = self._stack_component_states()
        stack_running = (
            robot_launch_running
            or realsense_camera_running
            or audio_capture_running
        )
        return {
            "dronecan_allocator_enabled": self.start_dronecan_allocator_on_startup,
            "dronecan_allocator_running": self.dronecan_allocator_running,
            "stack_running": stack_running,
            "stack_ready": (
                robot_launch_running
                and realsense_camera_running
                and audio_capture_running
            ),
            "robot_launch_running": robot_launch_running,
            "realsense_camera_running": realsense_camera_running,
            "audio_capture_running": audio_capture_running,
            "vision_yolo_running": self.vision_yolo_running,
            "vision_mission_running": self.vision_mission_running,
            "pinger_running": bool(self._pinger and self._pinger.is_running),
            "bag_running": bool(self._bag and self._bag.is_running),
            "bag_output": self._bag_output,
            "logs": list(self.logs)[-80:],
        }

    def stop_all(self) -> None:
        self.stop_bag()
        self.stop_vision()
        self.stop_pinger()
        self.stop_stack()
        self.stop_dronecan_allocator()

    @property
    def stack_running(self) -> bool:
        return bool(self._stack_process_groups())

    @property
    def stack_ready(self) -> bool:
        return (
            self.robot_launch_running
            and self.realsense_camera_running
            and self.audio_capture_running
        )

    @property
    def robot_launch_running(self) -> bool:
        return self._launch_running(
            self._stack,
            self.robot_package,
            self.robot_launch,
        )

    @property
    def realsense_camera_running(self) -> bool:
        return self._launch_running(
            self._realsense_camera,
            self.realsense_package,
            self.realsense_launch,
        )

    @property
    def audio_capture_running(self) -> bool:
        return self._launch_running(
            self._audio_capture,
            self.audio_capture_package,
            self.audio_capture_launch,
        )

    @property
    def dronecan_allocator_running(self) -> bool:
        return bool(self._dronecan_allocator and self._dronecan_allocator.is_running)

    @property
    def vision_yolo_running(self) -> bool:
        return bool(self._vision_yolo and self._vision_yolo.is_running)

    @property
    def vision_mission_running(self) -> bool:
        return bool(self._vision_mission and self._vision_mission.is_running)

    def stop_dronecan_allocator(self) -> None:
        if self._dronecan_allocator:
            self._dronecan_allocator.stop()

    def _uses_external_dronecan_allocator(self) -> bool:
        if not self.dronecan_allocator_running:
            return False
        if self.robot_package != "auv":
            return False
        return self.robot_launch in {"localization_test.launch.py", "rov_start.launch.py"}

    def _stack_process_groups(self) -> set[int]:
        groups: set[int] = set()
        launches = self._stack_launches()
        discovered = _matching_launch_process_groups_many(
            (package, launch_file)
            for _, package, launch_file in launches
        )
        for process, package, launch_file in launches:
            groups.update(discovered[(package, launch_file)])
            if process and process.pgid is not None and process.is_running:
                groups.add(process.pgid)
        return groups

    def _stack_component_states(self) -> tuple[bool, bool, bool]:
        launches = self._stack_launches()
        discovered = _matching_launch_process_groups_many(
            (package, launch_file)
            for _, package, launch_file in launches
        )
        states = tuple(
            bool(process and process.is_running)
            or bool(discovered[(package, launch_file)])
            for process, package, launch_file in launches
        )
        return states

    def _stack_launches(
        self,
    ) -> tuple[tuple[ManagedProcess | None, str, str], ...]:
        return (
            (self._stack, self.robot_package, self.robot_launch),
            (
                self._realsense_camera,
                self.realsense_package,
                self.realsense_launch,
            ),
            (
                self._audio_capture,
                self.audio_capture_package,
                self.audio_capture_launch,
            ),
        )

    @staticmethod
    def _launch_running(
        process: ManagedProcess | None,
        package: str,
        launch_file: str,
    ) -> bool:
        return bool(process and process.is_running) or bool(
            _matching_launch_process_groups(package, launch_file)
        )

    def _stop_orphaned_stack_groups(self) -> None:
        groups = self._stack_process_groups()
        for pgid in sorted(groups):
            self._log(f"[robot_stack] stopping orphaned process group {pgid}")
            _kill_process_group(pgid, signal.SIGTERM)
        for pgid in sorted(groups):
            if _wait_for_process_group_exit(pgid, timeout=5.0):
                continue
            self._log(f"[robot_stack] force killing orphaned process group {pgid}")
            _kill_process_group(pgid, signal.SIGKILL)
            _wait_for_process_group_exit(pgid, timeout=2.0)

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        entry = f"{timestamp} {message}"
        self.logs.append(entry)
        print(entry, file=sys.stdout, flush=True)


def _matching_launch_process_groups(robot_package: str, robot_launch: str) -> set[int]:
    key = (robot_package, robot_launch)
    return _matching_launch_process_groups_many((key,))[key]


def _matching_launch_process_groups_many(
    launches: Iterable[tuple[str, str]],
) -> dict[tuple[str, str], set[int]]:
    launch_keys = tuple(dict.fromkeys(launches))
    groups = {key: set() for key in launch_keys}
    current_pid = os.getpid()
    for pid, cmdline in _iter_process_cmdlines():
        if pid == current_pid:
            continue
        for package, launch_file in launch_keys:
            if not _is_matching_ros2_launch(cmdline, package, launch_file):
                continue
            try:
                groups[(package, launch_file)].add(os.getpgid(pid))
            except ProcessLookupError:
                pass
    return groups


def _ros2_launch_command(
    package: str,
    launch_file: str,
    launch_args: dict[str, str] | None,
) -> list[str]:
    cmd = ["ros2", "launch", package, launch_file]
    for key, value in (launch_args or {}).items():
        clean_value = str(value)
        if clean_value != "":
            cmd.append(f"{key}:={clean_value}")
    return cmd


def _is_matching_ros2_launch(
    cmdline: list[str],
    robot_package: str,
    robot_launch: str,
) -> bool:
    for index, arg in enumerate(cmdline[:-3]):
        if os.path.basename(arg) != "ros2":
            continue
        if cmdline[index + 1 : index + 4] == ["launch", robot_package, robot_launch]:
            return True
    return False


def _iter_process_cmdlines() -> Iterable[tuple[int, list[str]]]:
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if not raw:
            continue
        cmdline = [part.decode(errors="replace") for part in raw.split(b"\0") if part]
        if cmdline:
            yield int(entry.name), cmdline


def _kill_process_group(pgid: int, sig: signal.Signals) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        return


def _signal_process(pid: int, sig: signal.Signals) -> None:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        return


def _wait_for_process_group_exit(pgid: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _process_group_has_live_members(pgid):
            return True
        time.sleep(0.1)
    return not _process_group_has_live_members(pgid)


def _wait_for_pids_to_exit(
    pids: Iterable[int],
    executable: str,
    marker: str,
    timeout: float,
) -> bool:
    pid_set = set(pids)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _matching_pids_still_live(pid_set, executable, marker):
            return True
        time.sleep(0.1)
    return not _matching_pids_still_live(pid_set, executable, marker)


def _wait_for_new_matching_pids(
    groups: set[int],
    old_pids: Iterable[int],
    executable: str,
    marker: str,
    timeout: float,
) -> list[int]:
    old_pid_set = set(old_pids)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pids = [
            pid for pid in _matching_process_pids(groups, executable, marker)
            if pid not in old_pid_set
        ]
        if pids:
            return pids
        time.sleep(0.1)
    return []


def _matching_pids_still_live(pids: set[int], executable: str, marker: str) -> bool:
    for pid, cmdline in _iter_process_cmdlines():
        if pid not in pids or not cmdline:
            continue
        if os.path.basename(cmdline[0]) != executable:
            continue
        if marker and not any(marker in part for part in cmdline):
            continue
        return True
    return False


def _matching_process_pids(groups: set[int], executable: str, marker: str) -> list[int]:
    pids: list[int] = []
    for pid, cmdline in _iter_process_cmdlines():
        try:
            pgid = os.getpgid(pid)
        except ProcessLookupError:
            continue
        if pgid not in groups:
            continue
        if not cmdline or os.path.basename(cmdline[0]) != executable:
            continue
        if marker and not any(marker in part for part in cmdline):
            continue
        pids.append(pid)
    return sorted(pids)


def _process_group_has_live_members(pgid: int) -> bool:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            pid = int(entry.name)
            if os.getpgid(pid) != pgid:
                continue
            stat = (entry / "stat").read_text(encoding="utf-8", errors="replace")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        parts = stat.rsplit(")", 1)
        if len(parts) != 2:
            continue
        fields = parts[1].strip().split()
        if fields and fields[0] != "Z":
            return True
    return False
