"""Autotune workflow controls for the GUI."""

from __future__ import annotations

import datetime as _dt

from .config import *
from .helpers import *
from .models import ControlCommands, RcReplaySample
from .node import UuvGuiNode
from .ros_tools import *
from .runtime import *
from .widgets import VirtualJoystick

class AutoTuneMixin:
    def _toggle_autotune_panel(self) -> None:
        show = not self.autotune_visible.get()
        self.autotune_visible.set(show)
        if self.autotune_frame is None:
            return
        if show:
            self.autotune_frame.grid()
            if self.autotune_toggle_button is not None:
                self.autotune_toggle_button.config(text="Hide auto tune")
        else:
            self.autotune_frame.grid_remove()
            if self.autotune_toggle_button is not None:
                self.autotune_toggle_button.config(text="Show auto tune")

    def _autotune_running(self) -> bool:
        return self._autotune_process is not None and self._autotune_process.poll() is None

    def _set_autotune_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.autotune_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.autotune_status_var.set(text))
        except Exception:
            pass

    def _browse_autotune_bag(self) -> None:
        initial_dir = str(DEFAULT_AUTOTUNE_BAG.parent if DEFAULT_AUTOTUNE_BAG.parent.exists() else APP_ROOT)
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Select real ROS2 .db3 bag",
            initialdir=initial_dir,
            filetypes=(("ROS2 sqlite bag", "*.db3"), ("All files", "*")),
        )
        if selected:
            self.autotune_bag_var.set(selected)
            self._set_autotune_status("autotune: bag selected")

    @staticmethod
    def _read_float_var(var: tk.StringVar, default: float, lower: float, upper: float) -> float:
        try:
            value = float(var.get())
        except ValueError:
            value = default
        value = clamp(value, lower, upper)
        var.set(f"{value:g}")
        return value

    @staticmethod
    def _read_int_var(var: tk.StringVar, default: int, lower: int, upper: int) -> int:
        try:
            value = int(float(var.get()))
        except ValueError:
            value = default
        value = int(clamp(value, lower, upper))
        var.set(str(value))
        return value

    def _show_autotune_monitor(self) -> None:
        if self.autotune_monitor_window is not None and self.autotune_monitor_window.winfo_exists():
            self.autotune_monitor_window.deiconify()
            self.autotune_monitor_window.lift()
            return

        win = tk.Toplevel(self.root)
        win.title("UUV Auto Tune Monitor")
        win.geometry("860x620")
        win.minsize(720, 480)
        win.protocol("WM_DELETE_WINDOW", self._hide_autotune_monitor)
        self.autotune_monitor_window = win

        outer = ttk.Frame(win, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, textvariable=self.autotune_monitor_status_var, anchor="w").grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(header, text="Open output", command=self._open_autotune_output).grid(row=0, column=1, padx=(8, 0))

        progress = ttk.Progressbar(
            outer,
            variable=self.autotune_monitor_progress_var,
            maximum=100.0,
            mode="determinate",
        )
        progress.grid(row=1, column=0, sticky="ew", pady=(6, 8))

        body = ttk.PanedWindow(outer, orient=tk.VERTICAL)
        body.grid(row=2, column=0, sticky="nsew")

        top = ttk.Frame(body)
        top.columnconfigure(0, weight=1)
        top.rowconfigure(0, weight=1)
        body.add(top, weight=2)

        self.autotune_tree = ttk.Treeview(
            top,
            columns=("status", "score", "details"),
            show="tree headings",
            height=7,
        )
        self.autotune_tree.heading("#0", text="candidate")
        self.autotune_tree.heading("status", text="status")
        self.autotune_tree.heading("score", text="score")
        self.autotune_tree.heading("details", text="details")
        self.autotune_tree.column("#0", width=170, stretch=False)
        self.autotune_tree.column("status", width=95, stretch=False)
        self.autotune_tree.column("score", width=90, stretch=False, anchor=tk.E)
        self.autotune_tree.column("details", width=420, stretch=True)
        tree_scroll = ttk.Scrollbar(top, orient=tk.VERTICAL, command=self.autotune_tree.yview)
        self.autotune_tree.configure(yscrollcommand=tree_scroll.set)
        self.autotune_tree.grid(row=0, column=0, sticky="nsew")
        tree_scroll.grid(row=0, column=1, sticky="ns")

        self.autotune_chart_canvas = tk.Canvas(top, height=170, bg="#101827", highlightthickness=0)
        self.autotune_chart_canvas.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.autotune_chart_canvas.bind("<Configure>", lambda _event: self._redraw_autotune_chart())

        log_frame = ttk.LabelFrame(body, text="Live Log", padding=4)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        body.add(log_frame, weight=3)

        self.autotune_log_text = tk.Text(
            log_frame,
            height=10,
            wrap="none",
            bg="#111111",
            fg="#e5e7eb",
            insertbackground="#e5e7eb",
            font=("Menlo", 11),
        )
        log_y = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.autotune_log_text.yview)
        log_x = ttk.Scrollbar(log_frame, orient=tk.HORIZONTAL, command=self.autotune_log_text.xview)
        self.autotune_log_text.configure(yscrollcommand=log_y.set, xscrollcommand=log_x.set)
        self.autotune_log_text.grid(row=0, column=0, sticky="nsew")
        log_y.grid(row=0, column=1, sticky="ns")
        log_x.grid(row=1, column=0, sticky="ew")

        for name in self._autotune_candidate_order:
            self._upsert_autotune_candidate_row(name)
        self._refresh_autotune_progress()
        self._redraw_autotune_chart()

    def _hide_autotune_monitor(self) -> None:
        if self.autotune_monitor_window is not None and self.autotune_monitor_window.winfo_exists():
            self.autotune_monitor_window.withdraw()

    def _reset_autotune_monitor(self, out_dir: Path) -> None:
        self._autotune_candidate_order = []
        self._autotune_candidate_rows = {}
        self._autotune_current_candidate = None
        self.autotune_monitor_progress_var.set(0.0)
        self.autotune_monitor_status_var.set(f"autotune monitor: starting -> {out_dir.name}")
        if self.autotune_tree is not None:
            for item in self.autotune_tree.get_children():
                self.autotune_tree.delete(item)
        if self.autotune_log_text is not None:
            self.autotune_log_text.configure(state=tk.NORMAL)
            self.autotune_log_text.delete("1.0", tk.END)
            self.autotune_log_text.configure(state=tk.DISABLED)
        self._redraw_autotune_chart()

    def _post_autotune_monitor_line(self, line: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self._handle_autotune_monitor_line(line)
            return
        try:
            self.root.after(0, lambda: self._handle_autotune_monitor_line(line))
        except Exception:
            pass

    def _append_autotune_monitor_log(self, line: str) -> None:
        if self.autotune_log_text is None:
            return
        try:
            self.autotune_log_text.configure(state=tk.NORMAL)
            self.autotune_log_text.insert(tk.END, line + "\n")
            line_count = int(self.autotune_log_text.index("end-1c").split(".")[0])
            if line_count > 1200:
                self.autotune_log_text.delete("1.0", "200.0")
            self.autotune_log_text.see(tk.END)
            self.autotune_log_text.configure(state=tk.DISABLED)
        except Exception:
            pass

    def _handle_autotune_monitor_line(self, line: str) -> None:
        self._append_autotune_monitor_log(line)

        candidates_match = re.match(r"^\[autotune\]\s+candidates=(.+)$", line)
        if candidates_match:
            names = [part.strip() for part in candidates_match.group(1).split(",") if part.strip()]
            self._autotune_candidate_order = names
            for name in names:
                row = self._autotune_candidate_rows.setdefault(
                    name,
                    {"status": "queued", "score": math.nan, "details": ""},
                )
                row.setdefault("status", "queued")
                self._upsert_autotune_candidate_row(name)
            self.autotune_monitor_status_var.set(f"autotune monitor: queued {len(names)} candidates")
            self._refresh_autotune_progress()
            self._redraw_autotune_chart()
            return

        candidate_match = re.match(r"^\[autotune\]\s+candidate\s+([^:]+):\s+(.+)$", line)
        if candidate_match:
            name = candidate_match.group(1).strip()
            detail = candidate_match.group(2).strip()
            if detail == "start":
                self._autotune_current_candidate = name
                self._set_autotune_candidate(name, "running", math.nan, "running closed-loop replay")
                self.autotune_monitor_status_var.set(f"autotune monitor: running {name}")
            elif detail.startswith("score="):
                try:
                    score = float(detail.split("=", 1)[1])
                except ValueError:
                    score = math.nan
                self._set_autotune_candidate(name, "ok", score, "comparison finished")
                self.autotune_monitor_status_var.set(f"autotune monitor: {name} score={score:.4f}")
            elif detail.startswith("failed"):
                self._set_autotune_candidate(name, "failed", math.inf, detail)
                self.autotune_monitor_status_var.set(f"autotune monitor: {name} failed")
            else:
                self._set_autotune_candidate(name, "running", math.nan, detail)
            self._refresh_autotune_progress()
            self._redraw_autotune_chart()
            return

        best_match = re.match(r"^\[autotune\]\s+best=([^\s]+)\s+score=([0-9.eE+-]+)", line)
        if best_match:
            name = best_match.group(1)
            score = float(best_match.group(2))
            self.autotune_monitor_status_var.set(f"autotune monitor: best {name} score={score:.4f}")
            self._set_autotune_candidate(name, "best", score, "selected best candidate")
            self._refresh_autotune_progress()
            self._redraw_autotune_chart()
            return

        if line.startswith("[autotune] out="):
            self.autotune_monitor_status_var.set(f"autotune monitor: output {line.split('=', 1)[1]}")
            return

        if self._autotune_current_candidate and line.startswith("["):
            short_line = line if len(line) <= 120 else f"{line[:117]}..."
            self.autotune_monitor_status_var.set(
                f"autotune monitor: {self._autotune_current_candidate} | {short_line}"
            )

    def _set_autotune_candidate(self, name: str, status: str, score: float, details: str) -> None:
        if name not in self._autotune_candidate_order:
            self._autotune_candidate_order.append(name)
        row = self._autotune_candidate_rows.setdefault(
            name,
            {"status": "queued", "score": math.nan, "details": ""},
        )
        row["status"] = status
        if math.isfinite(score) or score == math.inf:
            row["score"] = score
        row["details"] = details
        self._upsert_autotune_candidate_row(name)

    def _upsert_autotune_candidate_row(self, name: str) -> None:
        if self.autotune_tree is None:
            return
        row = self._autotune_candidate_rows.get(name, {})
        status = str(row.get("status", "queued"))
        score = row.get("score", math.nan)
        score_text = "n/a"
        if isinstance(score, (int, float)):
            if math.isfinite(float(score)):
                score_text = f"{float(score):.4f}"
            elif float(score) == math.inf:
                score_text = "failed"
        details = str(row.get("details", ""))
        if self.autotune_tree.exists(name):
            self.autotune_tree.item(name, text=name, values=(status, score_text, details))
        else:
            self.autotune_tree.insert("", tk.END, iid=name, text=name, values=(status, score_text, details))

    def _refresh_autotune_progress(self) -> None:
        total = len(self._autotune_candidate_order)
        completed = 0
        for row in self._autotune_candidate_rows.values():
            if row.get("status") in {"ok", "failed", "best"}:
                completed += 1
        value = 0.0 if total <= 0 else 100.0 * completed / total
        self.autotune_monitor_progress_var.set(value)

    def _redraw_autotune_chart(self) -> None:
        canvas = self.autotune_chart_canvas
        if canvas is None:
            return
        width = max(canvas.winfo_width(), 500)
        height = max(canvas.winfo_height(), 160)
        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill="#101827", outline="")
        canvas.create_text(
            12,
            10,
            anchor="nw",
            fill="#d1d5db",
            font=("Menlo", 11, "bold"),
            text="Auto tune score by candidate (lower is better)",
        )
        names = self._autotune_candidate_order
        if not names:
            canvas.create_text(
                width / 2,
                height / 2,
                fill="#9ca3af",
                font=("Menlo", 12),
                text="waiting for candidate list...",
            )
            return

        finite_scores = [
            float(self._autotune_candidate_rows.get(name, {}).get("score"))
            for name in names
            if isinstance(self._autotune_candidate_rows.get(name, {}).get("score"), (int, float))
            and math.isfinite(float(self._autotune_candidate_rows.get(name, {}).get("score")))
        ]
        max_score = max(finite_scores) if finite_scores else 1.0
        chart_top = 34
        chart_bottom = height - 34
        chart_left = 34
        chart_right = width - 16
        bar_gap = 8
        bar_width = max(18, (chart_right - chart_left - bar_gap * max(len(names) - 1, 0)) / max(len(names), 1))

        canvas.create_line(chart_left, chart_bottom, chart_right, chart_bottom, fill="#334155")
        for idx, name in enumerate(names):
            row = self._autotune_candidate_rows.get(name, {})
            status = row.get("status", "queued")
            score = row.get("score", math.nan)
            x0 = chart_left + idx * (bar_width + bar_gap)
            x1 = min(x0 + bar_width, chart_right)
            color = "#475569"
            if status == "running":
                color = "#38bdf8"
            elif status == "ok":
                color = "#22c55e"
            elif status == "best":
                color = "#facc15"
            elif status == "failed":
                color = "#ef4444"

            if isinstance(score, (int, float)) and math.isfinite(float(score)):
                frac = clamp(float(score) / max(max_score, 1.0e-9), 0.02, 1.0)
                y0 = chart_bottom - frac * (chart_bottom - chart_top)
                canvas.create_rectangle(x0, y0, x1, chart_bottom, fill=color, outline="")
                canvas.create_text(
                    (x0 + x1) / 2,
                    max(chart_top + 8, y0 - 8),
                    fill="#e5e7eb",
                    font=("Menlo", 9),
                    text=f"{float(score):.2f}",
                )
            else:
                y0 = chart_bottom - 5
                canvas.create_rectangle(x0, y0, x1, chart_bottom, fill=color, outline="")

            label = name if len(name) <= 14 else f"{name[:12]}.."
            canvas.create_text((x0 + x1) / 2, height - 18, fill="#cbd5e1", font=("Menlo", 9), text=label)

    def _finish_autotune_monitor(self, rc: int, last_line: str) -> None:
        if rc == 0:
            self.autotune_monitor_status_var.set("autotune monitor: finished")
        elif rc == 130:
            self.autotune_monitor_status_var.set("autotune monitor: stopped")
        else:
            text = last_line if last_line else f"rc={rc}"
            if len(text) > 140:
                text = f"{text[:137]}..."
            self.autotune_monitor_status_var.set(f"autotune monitor: failed | {text}")

    def _start_autotune(self) -> None:
        if self._autotune_running():
            self._set_autotune_status("autotune: already running")
            return
        if self._sim_stack_running():
            self._set_autotune_status("autotune: stopping sim stack first")
            self._set_sim_stack_status("sim: stopping for autotune")
            self.node.push_event("autotune requested: stopping running sim stack")
            self._terminate_sim_stack_process()
        if not AUTOTUNE_SCRIPT.exists():
            self._set_autotune_status(f"autotune script missing: {AUTOTUNE_SCRIPT}")
            return

        bag_path = Path(self.autotune_bag_var.get()).expanduser()
        if not bag_path.exists():
            self._set_autotune_status(f"autotune: bag path missing: {bag_path}")
            return

        if self._rc_replay_running():
            self._stop_rc_replay()
        self.rc_override_enabled.set(False)
        self.node.publish_rc_release()

        start_s = self._read_float_var(self.autotune_start_var, 60.0, 0.0, 10000.0)
        duration_s = self._read_float_var(self.autotune_duration_var, 120.0, 10.0, 600.0)
        max_candidates = self._read_int_var(self.autotune_candidates_var, 15, 1, 30)
        servo_scale = self._read_float_var(self.autotune_servo_scale_var, 0.58, 0.1, 2.0)
        tune_mode = self.autotune_mode_var.get().strip() or "plant-rc-out"
        candidate_set = self.autotune_candidate_set_var.get().strip() or "ellipsoid5"
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = APP_ROOT / "document" / "docsource" / f"gui_autotune_{stamp}"
        self.autotune_out_dir_var.set(str(out_dir))
        self._show_autotune_monitor()
        self._reset_autotune_monitor(out_dir)

        autotune_python = resolve_autotune_python()
        cmd = [
            autotune_python,
            str(AUTOTUNE_SCRIPT),
            "--bag",
            str(bag_path),
            "--out-root",
            str(out_dir),
            "--start-offset-s",
            f"{start_s:g}",
            "--duration-s",
            f"{duration_s:g}",
            "--sitl-servo-scale",
            f"{servo_scale:g}",
            "--max-candidates",
            str(max_candidates),
            "--tune-mode",
            tune_mode,
            "--candidate-set",
            candidate_set,
        ]
        if self.autotune_apply_best_var.get():
            cmd.append("--apply-best")
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(APP_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                start_new_session=True,
            )
        except Exception as exc:
            self._set_autotune_status(f"autotune start failed: {exc}")
            self.autotune_monitor_status_var.set(f"autotune monitor: start failed | {exc}")
            return

        self._autotune_process = proc
        self._set_autotune_status(f"autotune: running -> {out_dir.name}")
        self.node.push_event(f"autotune start: {out_dir.name}")
        self._autotune_thread = threading.Thread(
            target=self._read_autotune_output,
            args=(proc,),
            daemon=True,
        )
        self._autotune_thread.start()

    def _read_autotune_output(self, proc: subprocess.Popen[str]) -> None:
        last_line = ""
        try:
            assert proc.stdout is not None
            for raw_line in proc.stdout:
                line = raw_line.strip()
                if not line:
                    continue
                last_line = line
                self._post_autotune_monitor_line(line)
                if line.startswith("[autotune]"):
                    short_line = line if len(line) <= 150 else f"{line[:147]}..."
                    self.node.push_event(short_line)
                    self._set_autotune_status(f"autotune: {short_line.removeprefix('[autotune] ').strip()}")
            rc = proc.wait()
        except Exception as exc:
            rc = -1
            last_line = f"reader failed: {exc}"

        def finish() -> None:
            if self._autotune_process is proc:
                self._autotune_process = None
            if rc == 0:
                self.autotune_status_var.set("autotune: finished")
                self.node.push_event("autotune finished")
            elif rc == 130:
                self.autotune_status_var.set("autotune: stopped")
                self.node.push_event("autotune stopped")
            else:
                text = last_line if last_line else f"rc={rc}"
                self.autotune_status_var.set(f"autotune failed: {text}")
                self.node.push_event(f"autotune failed: {text}")
            self._finish_autotune_monitor(rc, last_line)

        try:
            self.root.after(0, finish)
        except Exception:
            pass

    def _stop_autotune(self) -> None:
        proc = self._autotune_process
        if proc is None or proc.poll() is not None:
            self._set_autotune_status("autotune: not running")
            return
        try:
            self._terminate_process_group(proc)
            self._set_autotune_status("autotune: stopping")
            self.node.push_event("autotune stop requested")
        except Exception as exc:
            self._set_autotune_status(f"autotune stop failed: {exc}")

    def _open_autotune_output(self) -> None:
        path_text = self.autotune_out_dir_var.get()
        if not path_text:
            self._set_autotune_status("autotune: no output yet")
            return
        path = Path(path_text).expanduser()
        if not path.exists():
            self._set_autotune_status(f"autotune output missing: {path}")
            return
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        try:
            subprocess.Popen([opener, str(path)])
        except Exception as exc:
            self._set_autotune_status(f"open output failed: {exc}")
