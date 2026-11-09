"""Discover completed Real2Sim models and launch isolated offline replay viewers."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from pathlib import Path
import subprocess
import sys
import tempfile
import threading

from .config_paths import APP_ROOT, SIM_STACK_DIR
from .process_termination import terminate_process_group


class WebReal2Sim:
    """Manage a single viewer process, accepting only discovered output IDs."""

    def __init__(self, root: Path | None = None, workspace: Path | None = None) -> None:
        configured = os.environ.get("UUV_REAL2SIM_OUTPUT_ROOT")
        self.workspace = Path(workspace or APP_ROOT).resolve()
        self.root = Path(root or configured or self.workspace / "outputs").resolve()
        self._lock = threading.Lock()
        self._process = None
        self._log = None
        self._stopped = False
        self._build_process = None
        self._build_log = None
        self._build_output = None
        self._build_stopped = False

    def build(self, payload: dict) -> dict:
        """Start one bounded, headless calibration from a workspace package."""
        package_name = payload.get("package")
        output_name = payload.get("output_name")
        if not isinstance(package_name, str) or not package_name.strip() or Path(package_name).is_absolute():
            raise ValueError("Package must be a workspace-relative path")
        package = (self.workspace / package_name).resolve()
        if not package.is_relative_to(self.workspace) or not (package / "vehicle.json").is_file():
            raise ValueError("Package must be a workspace directory containing vehicle.json")
        if not isinstance(output_name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", output_name):
            raise ValueError("Output name must use letters, numbers, underscores, or hyphens")
        output = self.root / output_name
        with self._lock:
            if self._build_process is not None and self._build_process.poll() is None:
                raise ValueError("A Real2Sim build is already running")
            self.root.mkdir(parents=True, exist_ok=True)
            if output.exists() or output.is_symlink():
                raise ValueError("Output already exists; choose a new name")
            if self._build_log is not None:
                self._build_log.close()
            self._build_log = tempfile.TemporaryFile(mode="w+b")
            local_python = Path.home() / ".local/share/uuv-real2sim/venv/bin/python"
            python = os.environ.get("UUV_REAL2SIM_PYTHON") or (str(local_python) if local_python.is_file() else sys.executable)
            command = [python, str(SIM_STACK_DIR / "tools" / "real2sim_auto.py"), "run", str(package),
                       "--output", str(output), "--headless", "--jobs", "2", "--max-evaluations", "35"]
            self._build_output = output
            self._build_stopped = False
            try:
                self._build_process = subprocess.Popen(command, cwd=self.workspace, stdout=self._build_log,
                                                       stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                self._build_log.close()
                self._build_log = None
                self._build_process = None
                raise ValueError(f"Could not start Real2Sim: {exc}") from exc
            return {"running": True, "status": "Real2Sim build started", "output": output_name, "success": None}

    def build_status(self) -> dict:
        with self._lock:
            if self._build_process is None:
                return {"running": False, "status": "Build idle", "log": "", "output": None, "success": None}
            code = self._build_process.poll()
            running = code is None
            success = None if running else code == 0 and (self._build_output / "report.json").is_file()
            status = ("Real2Sim build running" if running else "Real2Sim build stopped" if self._build_stopped
                      else "Real2Sim build complete" if success else f"Real2Sim build failed (exit {code})")
            log = ""
            if self._build_log is not None:
                size = os.fstat(self._build_log.fileno()).st_size
                log = os.pread(self._build_log.fileno(), min(size, 16000), max(0, size - 16000)).decode(
                    "utf-8", errors="replace"
                )
            return {"running": running, "status": status, "log": log,
                    "output": self._build_output.name, "success": success}

    def build_stop(self) -> dict:
        with self._lock:
            if self._build_process is not None and self._build_process.poll() is None:
                self._build_stopped = True
                terminate_process_group(self._build_process)
            return {"running": False, "status": "Real2Sim build stopped"}

    def vehicles(self) -> list[dict]:
        if not self.root.is_dir():
            return []
        result = []
        for path in sorted(self.root.rglob("report.json")):
            try:
                output = path.parent.resolve()
                if not output.is_relative_to(self.root):
                    continue
                state = json.loads(path.read_text(encoding="utf-8"))
                sensors = json.loads((output / "sensor_config.json").read_text(encoding="utf-8"))
                if not (isinstance(state, dict) and isinstance(sensors, dict)
                        and isinstance(state.get("vehicle"), str)
                        and isinstance(state.get("trials"), dict) and state["trials"]
                        and isinstance(state.get("variants"), dict) and "candidate" in state["variants"]
                        and all((output / name).is_file() for name in
                                ("vehicle.json", "profiles.json", "thruster_performance.json"))
                        and isinstance(state.get("scene"), str)
                        and (output / state["scene"]).resolve().is_relative_to(output)
                        and (output / state["scene"]).is_file()):
                    continue
                validation = state.get("validation")
                validation = validation if isinstance(validation, dict) else {}
                relative = output.relative_to(self.root).as_posix()
                identity = hashlib.sha256(relative.encode("utf-8")).hexdigest()[:24]
                result.append({"id": identity, "name": state["vehicle"], "output": relative, "sensors": list(sensors),
                               "trials": list(state["trials"]), "profiles": list(state["variants"]),
                               "needs_review": bool(validation.get("candidate_requires_review", False))})
            except (OSError, ValueError, TypeError):
                continue
        return result

    def _resolve(self, identity: str) -> tuple[Path, dict]:
        # IDs are generated from relative paths; no user-provided path reaches a subprocess.
        for path in self.root.rglob("report.json") if self.root.is_dir() else ():
            output = path.parent.resolve()
            if output.is_relative_to(self.root):
                key = hashlib.sha256(output.relative_to(self.root).as_posix().encode("utf-8")).hexdigest()[:24]
                if key == identity:
                    vehicle = next((item for item in self.vehicles() if item["id"] == key), None)
                    if vehicle is not None:
                        return output, vehicle
        raise ValueError("Unknown or incomplete Real2Sim vehicle")

    def start(self, payload: dict) -> dict:
        identity = payload.get("id")
        if not isinstance(identity, str):
            raise ValueError("A Real2Sim vehicle ID is required")
        output, vehicle = self._resolve(identity)
        trial = payload.get("trial") or vehicle["trials"][0]
        profile = payload.get("profile") or "candidate"
        if trial not in vehicle["trials"] or profile not in vehicle["profiles"]:
            raise ValueError("Unknown trial or profile")
        try:
            rate = float(payload.get("playback_rate", 1.0))
        except (TypeError, ValueError) as exc:
            raise ValueError("Invalid playback rate") from exc
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("Playback rate must be positive and finite")
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise ValueError("A Real2Sim viewer is already running")
            if self._log is not None:
                self._log.close()
            self._log = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")
            command = [sys.executable, str(SIM_STACK_DIR / "tools" / "real2sim_auto.py"), "view", str(output),
                       "--trial", trial, "--profile", profile, "--playback-rate", str(rate)]
            self._stopped = False
            try:
                self._process = subprocess.Popen(command, cwd=APP_ROOT, stdout=self._log,
                                                 stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                self._log.close()
                self._log = None
                raise ValueError(f"Could not start viewer: {exc}") from exc
            return {"running": True, "status": "Offline Real2Sim viewer started", "pid": self._process.pid}

    def status(self) -> dict:
        with self._lock:
            if self._process is None:
                return {"running": False, "status": "Viewer idle"}
            code = self._process.poll()
            if code is None:
                return {"running": True, "status": "Offline Real2Sim viewer running"}
            if self._stopped:
                return {"running": False, "status": "Viewer stopped"}
            message = "Viewer finished" if code == 0 else f"Viewer failed (exit {code})"
            if code and self._log is not None:
                self._log.flush()
                self._log.seek(0)
                tail = self._log.read().splitlines()[-1:]
                if tail:
                    message += f": {tail[0]}"
            return {"running": False, "status": message}

    def stop(self) -> dict:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                self._stopped = True
                terminate_process_group(self._process)
            return {"running": False, "status": "Viewer stopped"}
