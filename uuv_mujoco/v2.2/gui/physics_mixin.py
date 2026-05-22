"""Physics parameter editor controls for the GUI."""

from __future__ import annotations

import datetime as _dt

from .config import *
from .helpers import *
from .models import ControlCommands, RcReplaySample
from .node import UuvGuiNode
from .ros_tools import *
from .runtime import *
from .widgets import VirtualJoystick

class PhysicsMixin:
    def _show_physics_window(self) -> None:
        if self.physics_window is not None and self.physics_window.winfo_exists():
            self._load_physics_params_into_fields(silent=True)
            self.physics_window.deiconify()
            self.physics_window.lift()
            return

        self._load_physics_params_into_fields(silent=True)
        win = tk.Toplevel(self.root)
        win.title("UUV Sim Param Tuning")
        win.geometry("980x640")
        win.minsize(860, 500)
        win.protocol("WM_DELETE_WINDOW", self._close_physics_window)
        self.physics_window = win

        outer = ttk.Frame(win, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(
            header,
            text=f"profile: {PHYSICS_PROFILE_NAME}   file: {PHYSICS_PROFILE_PATH}",
            anchor="w",
        ).grid(row=0, column=0, sticky="ew")
        ttk.Button(header, text="Reload", command=self._load_physics_params_into_fields).grid(
            row=0, column=1, padx=(8, 0)
        )
        ttk.Button(header, text="Apply", command=self._apply_physics_params).grid(
            row=0, column=2, padx=(4, 0)
        )
        ttk.Button(
            header,
            text="Apply + Restart",
            command=lambda: self._apply_physics_params(restart=True),
        ).grid(row=0, column=3, padx=(4, 0))

        ttk.Label(
            outer,
            textvariable=self.physics_status_var,
            anchor="w",
        ).grid(row=1, column=0, sticky="ew", pady=(6, 6))

        body = ttk.Frame(outer)
        body.grid(row=2, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        canvas = tk.Canvas(body, highlightthickness=0)
        scroll_y = ttk.Scrollbar(body, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=scroll_y.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        self.physics_canvas = canvas

        grid = ttk.Frame(canvas, padding=(0, 0, 6, 0))
        self.physics_scroll_frame = grid
        window_id = canvas.create_window((0, 0), window=grid, anchor="nw")
        grid.columnconfigure(1, weight=1)
        grid.columnconfigure(3, weight=2)
        grid.columnconfigure(4, weight=1)

        def update_scroll_region(_event: tk.Event | None = None) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def update_inner_width(event: tk.Event) -> None:
            canvas.itemconfigure(window_id, width=event.width)

        grid.bind("<Configure>", update_scroll_region)
        canvas.bind("<Configure>", update_inner_width)

        ttk.Label(grid, text="parameter", anchor="w").grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(grid, text="value", anchor="w").grid(row=0, column=1, sticky="ew", pady=(0, 4))
        ttk.Label(grid, text="key", anchor="w").grid(row=0, column=2, sticky="ew", padx=(8, 8), pady=(0, 4))
        ttk.Label(grid, text="what it changes", anchor="w").grid(row=0, column=3, sticky="ew", pady=(0, 4))
        ttk.Label(grid, text="current mode", anchor="w").grid(row=0, column=4, sticky="ew", padx=(8, 0), pady=(0, 4))

        for row_idx, spec in enumerate(PHYSICS_PARAM_SPECS, start=1):
            key = str(spec["key"])
            inactive = self._physics_param_inactive_in_current(key)
            foreground = "#94a3b8" if inactive else "#0f172a"
            ttk.Label(grid, text=str(spec["label"]), width=25, anchor="w", foreground=foreground).grid(
                row=row_idx, column=0, sticky="w", pady=2
            )
            entry_state = "readonly" if inactive else "normal"
            ttk.Entry(grid, textvariable=self.physics_param_vars[key], width=24, state=entry_state).grid(
                row=row_idx, column=1, sticky="ew", padx=(4, 8), pady=2
            )
            ttk.Label(grid, text=key, anchor="w", foreground="#64748b").grid(
                row=row_idx, column=2, sticky="w", padx=(0, 8), pady=2
            )
            ttk.Label(
                grid,
                text=str(spec["description"]),
                anchor="w",
                foreground="#475569",
                wraplength=300,
            ).grid(row=row_idx, column=3, sticky="ew", pady=2)
            ttk.Label(
                grid,
                text=self._physics_current_mode_status(key),
                anchor="w",
                foreground="#b45309" if inactive else "#166534",
                wraplength=180,
            ).grid(row=row_idx, column=4, sticky="ew", padx=(8, 0), pady=2)

        footer = ttk.Frame(outer)
        footer.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        footer.columnconfigure(0, weight=1)
        ttk.Label(
            footer,
            text=(
                "Only current-mode active rows are applied. Inactive rows are read-only because "
                "running MuJoCo current mode does not consume those parameters."
            ),
            anchor="w",
            foreground="#64748b",
        ).grid(row=0, column=0, sticky="ew")
        ttk.Button(footer, text="Close", command=self._close_physics_window).grid(row=0, column=1, padx=(8, 0))

    def _close_physics_window(self) -> None:
        if self.physics_window is not None and self.physics_window.winfo_exists():
            self.physics_window.destroy()
        self.physics_window = None
        self.physics_canvas = None
        self.physics_scroll_frame = None

    def _set_physics_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.physics_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.physics_status_var.set(text))
        except Exception:
            pass

    @staticmethod
    def _physics_param_inactive_in_current(key: str) -> bool:
        return key in CURRENT_MODE_INACTIVE_PHYSICS_KEYS

    @staticmethod
    def _physics_current_mode_status(key: str) -> str:
        reason = CURRENT_MODE_INACTIVE_PHYSICS_KEYS.get(key)
        if reason:
            return f"inactive: {reason}"
        return "active"

    @staticmethod
    def _format_physics_value(value: Any) -> str:
        if isinstance(value, (list, tuple)):
            return " ".join(f"{float(item):g}" for item in value)
        return f"{float(value):g}"

    @staticmethod
    def _get_nested_value(mapping: dict[str, Any], dotted_key: str) -> Any:
        current: Any = mapping
        for part in dotted_key.split("."):
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]
        return current

    @staticmethod
    def _set_nested_value(mapping: dict[str, Any], dotted_key: str, value: Any) -> None:
        current = mapping
        parts = dotted_key.split(".")
        for part in parts[:-1]:
            child = current.get(part)
            if not isinstance(child, dict):
                child = {}
                current[part] = child
            current = child
        current[parts[-1]] = value

    @staticmethod
    def _current_physics_profile(payload: dict[str, Any]) -> dict[str, Any]:
        profiles = payload.get("profiles")
        if isinstance(profiles, dict) and isinstance(profiles.get(PHYSICS_PROFILE_NAME), dict):
            return profiles[PHYSICS_PROFILE_NAME]
        profile = payload.get(PHYSICS_PROFILE_NAME)
        if isinstance(profile, dict):
            return profile
        raise KeyError(f"profile '{PHYSICS_PROFILE_NAME}' not found")

    def _load_physics_params_into_fields(self, silent: bool = False) -> None:
        try:
            payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
            profile = self._current_physics_profile(payload)
            for spec in PHYSICS_PARAM_SPECS:
                key = str(spec["key"])
                value = self._get_nested_value(profile, key)
                if value is None:
                    value = spec["default"]
                self.physics_param_vars[key].set(self._format_physics_value(value))
        except Exception as exc:
            self._set_physics_status(f"physics params: load failed: {exc}")
            return
        if not silent:
            self._set_physics_status("physics params: loaded")

    @staticmethod
    def _parse_physics_number(value: float, label: str) -> float:
        if not math.isfinite(value):
            raise ValueError(f"{label} must be finite")
        return value

    def _parse_physics_value(self, spec: dict[str, Any]) -> Any:
        key = str(spec["key"])
        raw = self.physics_param_vars[key].get().strip()
        label = str(spec["label"])
        kind = str(spec.get("kind", "scalar"))
        vector_match = re.fullmatch(r"vector(\d+)", kind)
        if vector_match:
            expected_len = int(vector_match.group(1))
            pieces = [piece for piece in re.split(r"[,\s]+", raw) if piece]
            if len(pieces) != expected_len:
                raise ValueError(f"{label} must have exactly {expected_len} numbers")
            values = [
                self._parse_physics_number(float(piece), f"{label}[{idx}]")
                for idx, piece in enumerate(pieces)
            ]
            self.physics_param_vars[key].set(self._format_physics_value(values))
            return values
        value = self._parse_physics_number(float(raw), label)
        self.physics_param_vars[key].set(self._format_physics_value(value))
        return value

    def _apply_physics_params(self, restart: bool = False) -> None:
        try:
            payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
            profile = self._current_physics_profile(payload)
            for spec in PHYSICS_PARAM_SPECS:
                key = str(spec["key"])
                if self._physics_param_inactive_in_current(key):
                    continue
                self._set_nested_value(profile, key, self._parse_physics_value(spec))
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_path = PHYSICS_PROFILE_PATH.with_name(f"{PHYSICS_PROFILE_PATH.name}.bak_gui_phys_{stamp}")
            shutil.copy2(PHYSICS_PROFILE_PATH, backup_path)
            PHYSICS_PROFILE_PATH.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            self._set_physics_status(f"physics params: apply failed: {exc}")
            self.node.push_event(f"physics params apply failed: {exc}")
            try:
                log_dir = SIM_STACK_DIR / "logs"
                log_dir.mkdir(parents=True, exist_ok=True)
                log_path = log_dir / "gui_physics_apply_errors.log"
                with log_path.open("a", encoding="utf-8") as log_file:
                    stamp = _dt.datetime.now().isoformat(timespec="seconds")
                    log_file.write(f"[{stamp}] apply failed: {exc!r}\n")
                    for spec in PHYSICS_PARAM_SPECS:
                        key = str(spec["key"])
                        try:
                            raw = self.physics_param_vars[key].get()
                        except Exception:
                            raw = "<unreadable>"
                        log_file.write(f"  {key}={raw!r}\n")
            except Exception:
                pass
            return

        self._load_physics_params_into_fields(silent=True)
        self.node.push_event(f"physics params applied: backup {backup_path.name}")
        if restart:
            self._set_physics_status("physics params: applied; restarting sim")
            self._restart_sim_stack_after_physics_apply()
        else:
            self._set_physics_status("physics params: applied to file; restart required for running sim")

    def _restart_sim_stack_after_physics_apply(self) -> None:
        if self._sim_stack_reset_thread is not None and self._sim_stack_reset_thread.is_alive():
            self._set_sim_stack_status("sim: restart already in progress")
            return
        self._set_sim_stack_status("sim: resetting for physics params")
        self.node.push_event("physics params restart requested")
        self._terminate_sim_stack_process()
        self._refresh_sim_stack_controls()

        def reset_and_start() -> None:
            if not RESET_SIM_STACK_SCRIPT.exists():
                self._set_sim_stack_status(f"reset script missing: {RESET_SIM_STACK_SCRIPT}")
                return
            if self._sim_stack_backend() == "docker":
                self.node.push_event("docker SITL stop requested for physics restart")
                self._stop_docker_sitl_blocking(timeout_s=20.0)
            cmd = [str(RESET_SIM_STACK_SCRIPT), "--sim-only"]
            try:
                proc = subprocess.run(
                    cmd,
                    cwd=str(SIM_STACK_DIR),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=30,
                    start_new_session=True,
                )
            except Exception as exc:
                self._set_sim_stack_status(f"sim reset failed: {exc}")
                self.node.push_event(f"physics params sim reset failed: {exc}")
                return

            for line in proc.stdout.splitlines():
                line = line.strip()
                if line.startswith("[reset]"):
                    short_line = line if len(line) <= 150 else f"{line[:147]}..."
                    self.node.push_event(short_line)
            if proc.returncode == 0 and not self._wait_for_external_sim_stack_exit(timeout_s=8.0):
                self._set_sim_stack_status("sim: external stack still running after reset")
                self.node.push_event("physics params restart blocked: external stack still running")
                try:
                    self.root.after(0, self._refresh_sim_stack_controls)
                except Exception:
                    pass
                return

            def finish() -> None:
                if self._closed:
                    return
                if proc.returncode != 0:
                    self._set_sim_stack_status(f"sim reset failed: rc={proc.returncode}")
                    return
                # The reset helper kills the whole external sim stack, including
                # processes that may not be direct children of the current GUI.
                # Drop any stale handle so the immediate restart cannot be
                # skipped by _sim_stack_running() during process cleanup races.
                self._sim_stack_process = None
                self._set_sim_stack_status("sim: starting with physics params")
                self._refresh_sim_stack_controls()
                self._start_sim_stack(extra_args=["--no-reset"])

            try:
                self.root.after(0, finish)
            except Exception:
                pass

        self._sim_stack_reset_thread = threading.Thread(target=reset_and_start, daemon=True)
        self._sim_stack_reset_thread.start()
