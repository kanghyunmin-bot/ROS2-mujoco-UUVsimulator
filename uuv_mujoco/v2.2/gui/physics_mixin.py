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
        win.geometry("820x640")
        win.minsize(720, 500)
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

        for row_idx, spec in enumerate(PHYSICS_PARAM_SPECS, start=1):
            key = str(spec["key"])
            ttk.Label(grid, text=str(spec["label"]), width=25, anchor="w").grid(
                row=row_idx, column=0, sticky="w", pady=2
            )
            ttk.Entry(grid, textvariable=self.physics_param_vars[key], width=24).grid(
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

        footer = ttk.Frame(outer)
        footer.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        footer.columnconfigure(0, weight=1)
        ttk.Label(
            footer,
            text="Changes are written to sim_profiles.json. Running MuJoCo must be restarted to reload physics parameters.",
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

    def _parse_physics_value(self, spec: dict[str, Any]) -> Any:
        key = str(spec["key"])
        raw = self.physics_param_vars[key].get().strip()
        lower = float(spec.get("lower", -math.inf))
        upper = float(spec.get("upper", math.inf))
        kind = str(spec.get("kind", "scalar"))
        vector_match = re.fullmatch(r"vector(\d+)", kind)
        if vector_match:
            expected_len = int(vector_match.group(1))
            pieces = [piece for piece in re.split(r"[,\s]+", raw) if piece]
            if len(pieces) != expected_len:
                raise ValueError(f"{spec['label']} must have exactly {expected_len} numbers")
            values = [clamp(float(piece), lower, upper) for piece in pieces]
            self.physics_param_vars[key].set(self._format_physics_value(values))
            return values
        value = clamp(float(raw), lower, upper)
        self.physics_param_vars[key].set(self._format_physics_value(value))
        return value

    def _apply_physics_params(self, restart: bool = False) -> None:
        try:
            payload = json.loads(PHYSICS_PROFILE_PATH.read_text(encoding="utf-8"))
            profile = self._current_physics_profile(payload)
            for spec in PHYSICS_PARAM_SPECS:
                self._set_nested_value(profile, str(spec["key"]), self._parse_physics_value(spec))
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
            return

        self._set_physics_status("physics params: applied; restart required for running sim")
        self.node.push_event(f"physics params applied: backup {backup_path.name}")
        if restart:
            self._restart_sim_stack_after_physics_apply()

    def _restart_sim_stack_after_physics_apply(self) -> None:
        self._set_sim_stack_status("sim: restarting with physics params")
        self.node.push_event("physics params restart requested")
        if self._sim_stack_running():
            self._terminate_sim_stack_process()

        def start_when_ports_release() -> None:
            if self._closed:
                return
            if self._sim_stack_running():
                self.root.after(500, start_when_ports_release)
                return
            self._start_sim_stack()

        self.root.after(1500, start_when_ports_release)
