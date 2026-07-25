"""Top-view editor for competition-course buoy and robot XY positions."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

from .buoy_layout_model import (
    BuoyLayoutItem,
    RobotSpawnItem,
    TANK_X_HALF_M,
    TANK_Y_HALF_M,
    clamp_xy,
    load_buoy_layout,
    load_robot_spawn,
    save_buoy_layout,
)
from .config import COURSE_SCENE_PATH
from .runtime import tk, ttk


CANVAS_WIDTH = 700
CANVAS_HEIGHT = 600
CANVAS_MARGIN = 34
POINT_RADIUS = 7
ROBOT_TARGET_ID = "__robot_spawn__"
TARGET_TAG_PREFIX = "layout_target:"


class BuoyLayoutEditor:
    def __init__(self, owner: Any, scene_path: Path = COURSE_SCENE_PATH):
        self.owner = owner
        self.scene_path = Path(scene_path)
        self.win: tk.Toplevel | None = None
        self.canvas: tk.Canvas | None = None
        self.tree: ttk.Treeview | None = None
        self.status_var = tk.StringVar(value="course layout: idle")
        self.selected_target: str | None = None
        self.items: list[BuoyLayoutItem] = []
        self.item_by_prefix: dict[str, BuoyLayoutItem] = {}
        self.robot_spawn: RobotSpawnItem | None = None
        self.robot_xy: tuple[float, float] | None = None
        self.positions: dict[str, tuple[float, float]] = {}
        self.marker_to_target: dict[int, str] = {}
        self.x_var = tk.StringVar(value="")
        self.y_var = tk.StringVar(value="")
        self.z_var = tk.StringVar(value="")
        self.layer_var = tk.StringVar(value="")
        self._selecting_tree = False
        self._scale = 1.0
        self._origin_px = (CANVAS_MARGIN, CANVAS_MARGIN)

    def show(self) -> None:
        if self.win is not None and self.win.winfo_exists():
            self.reload()
            self.win.deiconify()
            self.win.lift()
            return
        self._build_window()
        self.reload()

    def _build_window(self) -> None:
        win = tk.Toplevel(self.owner.root)
        win.title("Course XY Layout")
        win.geometry("1080x680")
        win.minsize(980, 600)
        win.protocol("WM_DELETE_WINDOW", self.close)
        self.win = win
        self.owner.buoy_layout_window = win

        outer = ttk.Frame(win, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=0)
        outer.rowconfigure(1, weight=1)

        self._build_header(outer)
        self._build_canvas(outer)
        self._build_side_panel(outer)
        ttk.Label(outer, textvariable=self.status_var, anchor="w", style="Status.TLabel").grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(6, 0),
        )

    def _build_header(self, outer: ttk.Frame) -> None:
        header = ttk.Frame(outer)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text=f"scene: {self.scene_path}", anchor="w").grid(row=0, column=0, sticky="ew")
        ttk.Button(header, text="Reload", command=self.reload).grid(row=0, column=1, padx=(8, 0))
        ttk.Button(header, text="Save XML", style="Accent.TButton", command=self.save).grid(
            row=0,
            column=2,
            padx=(4, 0),
        )
        ttk.Button(header, text="Save + Reset Sim", style="Danger.TButton", command=self.save_and_reset).grid(
            row=0,
            column=3,
            padx=(4, 0),
        )

    def _build_canvas(self, outer: ttk.Frame) -> None:
        canvas = tk.Canvas(
            outer,
            width=CANVAS_WIDTH,
            height=CANVAS_HEIGHT,
            bg="#e0f7ff",
            highlightthickness=1,
            highlightbackground="#94a3b8",
        )
        canvas.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        canvas.bind("<Configure>", lambda _event=None: self.redraw())
        canvas.bind("<Button-1>", self._on_canvas_press)
        canvas.bind("<B1-Motion>", self._on_canvas_drag)
        canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.canvas = canvas
        self.owner.buoy_layout_canvas = canvas

    def _build_side_panel(self, outer: ttk.Frame) -> None:
        side = ttk.Frame(outer)
        side.grid(row=1, column=1, sticky="ns")
        side.rowconfigure(0, weight=1)

        columns = ("layer", "x", "y", "z")
        tree = ttk.Treeview(side, columns=columns, show="tree headings", height=20, selectmode="browse")
        tree.heading("#0", text="Item")
        tree.heading("layer", text="Layer")
        tree.heading("x", text="X")
        tree.heading("y", text="Y")
        tree.heading("z", text="Z")
        tree.column("#0", width=72, stretch=False)
        tree.column("layer", width=70, stretch=False)
        tree.column("x", width=72, anchor="e", stretch=False)
        tree.column("y", width=72, anchor="e", stretch=False)
        tree.column("z", width=72, anchor="e", stretch=False)
        tree.grid(row=0, column=0, sticky="nsew")
        tree.bind("<<TreeviewSelect>>", self._on_tree_select)
        self.tree = tree

        edit = ttk.LabelFrame(side, text="Selected XY", padding=6)
        edit.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        edit.columnconfigure(1, weight=1)
        ttk.Label(edit, text="X").grid(row=0, column=0, sticky="w")
        ttk.Entry(edit, textvariable=self.x_var, width=10).grid(row=0, column=1, sticky="ew", padx=(6, 0))
        ttk.Label(edit, text="Y").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(edit, textvariable=self.y_var, width=10).grid(row=1, column=1, sticky="ew", padx=(6, 0), pady=(4, 0))
        ttk.Label(edit, text="Z fixed").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Label(edit, textvariable=self.z_var, anchor="e").grid(row=2, column=1, sticky="ew", padx=(6, 0), pady=(4, 0))
        ttk.Label(edit, text="Height").grid(row=3, column=0, sticky="w", pady=(4, 0))
        ttk.Label(edit, textvariable=self.layer_var, anchor="e").grid(
            row=3,
            column=1,
            sticky="ew",
            padx=(6, 0),
            pady=(4, 0),
        )
        ttk.Button(edit, text="Update Selected", command=self.update_selected_from_entries).grid(
            row=4,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(8, 0),
        )

    def reload(self, select: str | None = None) -> None:
        try:
            self.items = load_buoy_layout(self.scene_path)
            self.robot_spawn = load_robot_spawn(self.scene_path)
        except Exception as exc:
            self.status_var.set(f"course layout load failed: {exc}")
            return
        self.item_by_prefix = {item.prefix: item for item in self.items}
        self.positions = {item.prefix: (item.x, item.y) for item in self.items}
        self.robot_xy = (self.robot_spawn.x, self.robot_spawn.y)
        self._populate_tree()
        chosen = select or self.selected_target
        if not self._target_exists(chosen):
            chosen = ROBOT_TARGET_ID if self.robot_spawn is not None else None
        if not self._target_exists(chosen) and self.items:
            chosen = self.items[0].prefix
        if chosen:
            self.select(chosen)
        self.redraw()
        self.status_var.set(f"course layout: loaded {len(self.items)} buoys + robot spawn")
        self.owner.buoy_layout_status_var.set(f"course layout: loaded {len(self.items)} buoys + robot")

    def _populate_tree(self) -> None:
        if self.tree is None:
            return
        self.tree.delete(*self.tree.get_children())
        if self.robot_spawn is not None and self.robot_xy is not None:
            x, y = self.robot_xy
            self.tree.insert(
                "",
                tk.END,
                iid=ROBOT_TARGET_ID,
                text=self.robot_spawn.label,
                values=(self.robot_spawn.layer, f"{x:.2f}", f"{y:.2f}", f"{self.robot_spawn.z:.2f}"),
            )
        for item in self.items:
            x, y = self.positions[item.prefix]
            self.tree.insert(
                "",
                tk.END,
                iid=item.prefix,
                text=item.label,
                values=(item.layer, f"{x:.2f}", f"{y:.2f}", f"{item.z:.2f}"),
            )

    def redraw(self) -> None:
        if self.canvas is None:
            return
        self.canvas.delete("all")
        self.marker_to_target.clear()
        width = max(int(self.canvas.winfo_width()), 200)
        height = max(int(self.canvas.winfo_height()), 200)
        scale = min(
            (width - 2 * CANVAS_MARGIN) / (2 * TANK_X_HALF_M),
            (height - 2 * CANVAS_MARGIN) / (2 * TANK_Y_HALF_M),
        )
        self._scale = scale
        tank_w = 2 * TANK_X_HALF_M * scale
        tank_h = 2 * TANK_Y_HALF_M * scale
        left = (width - tank_w) / 2.0
        top = (height - tank_h) / 2.0
        self._origin_px = (left, top)
        right = left + tank_w
        bottom = top + tank_h

        self.canvas.create_rectangle(left, top, right, bottom, fill="#dff8ff", outline="#0f172a", width=2)
        self._draw_grid(left, top, right, bottom)
        self._draw_course_marks(left, top, right, bottom)
        for item in self.items:
            self._draw_buoy(item)
        self._draw_robot_spawn()

    def _draw_grid(self, left: float, top: float, right: float, bottom: float) -> None:
        if self.canvas is None:
            return
        for x in range(-15, 16, 5):
            px, _ = self.world_to_canvas(float(x), 0.0)
            self.canvas.create_line(px, top, px, bottom, fill="#b7dce8")
            self.canvas.create_text(px + 2, bottom + 12, text=str(x), anchor="nw", fill="#475569", font=("TkDefaultFont", 8))
        for y in range(-10, 11, 5):
            _, py = self.world_to_canvas(0.0, float(y))
            self.canvas.create_line(left, py, right, py, fill="#b7dce8")
            self.canvas.create_text(left - 6, py, text=str(y), anchor="e", fill="#475569", font=("TkDefaultFont", 8))
        cx, _ = self.world_to_canvas(0.0, 0.0)
        self.canvas.create_line(cx, top, cx, bottom, fill="#64748b", dash=(4, 4), width=2)

    def _draw_course_marks(self, left: float, top: float, right: float, bottom: float) -> None:
        if self.canvas is None:
            return
        self.canvas.create_text((left + right) / 4.0, top + 18, text="A course", fill="#0f172a", font=("TkDefaultFont", 11, "bold"))
        self.canvas.create_text((left + right) * 3.0 / 4.0, top + 18, text="B course", fill="#0f172a", font=("TkDefaultFont", 11, "bold"))
        for gx in (-5.25, 5.25):
            px, py = self.world_to_canvas(gx, 0.0)
            r = 0.75 * self._scale
            self.canvas.create_oval(px - r, py - r, px + r, py + r, outline="#0284c7", width=3)

    def _draw_buoy(self, item: BuoyLayoutItem) -> None:
        if self.canvas is None:
            return
        x, y = self.positions[item.prefix]
        px, py = self.world_to_canvas(x, y)
        radius = POINT_RADIUS + (2 if item.prefix == self.selected_target else 0)
        outline = "#020617" if item.prefix != self.selected_target else "#2563eb"
        width = 2 if item.fixed_underwater else 1
        marker = self.canvas.create_oval(
            px - radius,
            py - radius,
            px + radius,
            py + radius,
            fill=item.color_hex,
            outline=outline,
            width=width,
            tags=("buoy_marker", f"{TARGET_TAG_PREFIX}{item.prefix}"),
        )
        self.marker_to_target[marker] = item.prefix
        if item.fixed_underwater:
            self.canvas.create_oval(px - radius - 4, py - radius - 4, px + radius + 4, py + radius + 4, outline="#475569")
        self.canvas.create_text(
            px + radius + 5,
            py,
            text=item.label,
            anchor="w",
            fill="#0f172a",
            font=("TkDefaultFont", 8, "bold" if item.prefix == self.selected_target else "normal"),
            tags=("buoy_label", f"{TARGET_TAG_PREFIX}{item.prefix}"),
        )

    def _draw_robot_spawn(self) -> None:
        if self.canvas is None or self.robot_spawn is None or self.robot_xy is None:
            return
        x, y = self.robot_xy
        px, py = self.world_to_canvas(x, y)
        radius = POINT_RADIUS + 8 + (3 if self.selected_target == ROBOT_TARGET_ID else 0)
        outline = "#0f172a" if self.selected_target != ROBOT_TARGET_ID else "#f97316"
        tags = ("robot_spawn_marker", f"{TARGET_TAG_PREFIX}{ROBOT_TARGET_ID}")
        points = (
            px + radius,
            py,
            px - radius * 0.75,
            py - radius * 0.70,
            px - radius * 0.35,
            py,
            px - radius * 0.75,
            py + radius * 0.70,
        )
        marker = self.canvas.create_polygon(
            points,
            fill=self.robot_spawn.color_hex,
            outline=outline,
            width=2,
            tags=tags,
        )
        self.marker_to_target[marker] = ROBOT_TARGET_ID
        self.canvas.create_oval(
            px - radius - 4,
            py - radius - 4,
            px + radius + 4,
            py + radius + 4,
            outline="#1d4ed8",
            width=2,
            dash=(4, 3),
            tags=tags,
        )
        self.canvas.create_text(
            px + radius + 7,
            py,
            text=self.robot_spawn.label,
            anchor="w",
            fill="#0f172a",
            font=("TkDefaultFont", 9, "bold"),
            tags=tags,
        )

    def world_to_canvas(self, x: float, y: float) -> tuple[float, float]:
        left, top = self._origin_px
        return (
            left + (x + TANK_X_HALF_M) * self._scale,
            top + (TANK_Y_HALF_M - y) * self._scale,
        )

    def canvas_to_world(self, px: float, py: float) -> tuple[float, float]:
        left, top = self._origin_px
        x = (px - left) / self._scale - TANK_X_HALF_M
        y = TANK_Y_HALF_M - (py - top) / self._scale
        return clamp_xy(x, y)

    def _on_tree_select(self, _event=None) -> None:
        if self._selecting_tree:
            return
        if self.tree is None:
            return
        selected = self.tree.selection()
        if selected:
            self.select(str(selected[0]))

    def _on_canvas_press(self, event) -> None:
        target_id = self._target_at_canvas_event(event)
        if target_id:
            self.select(target_id)

    def _on_canvas_drag(self, event) -> None:
        if self.selected_target is None:
            return
        x, y = self.canvas_to_world(event.x, event.y)
        self._set_position(self.selected_target, x, y)
        self._update_entry_vars(self.selected_target)
        self.redraw()

    def _on_canvas_release(self, _event) -> None:
        if self.selected_target:
            self._refresh_tree_row(self.selected_target)

    def _target_at_canvas_event(self, event) -> str | None:
        if self.canvas is None:
            return None
        current = self.canvas.find_withtag("current")
        for item_id in current:
            tags = self.canvas.gettags(item_id)
            for tag in tags:
                if tag.startswith(TARGET_TAG_PREFIX):
                    return tag.removeprefix(TARGET_TAG_PREFIX)
        nearest = self.canvas.find_closest(event.x, event.y)
        if nearest:
            tags = self.canvas.gettags(nearest[0])
            for tag in tags:
                if tag.startswith(TARGET_TAG_PREFIX):
                    return tag.removeprefix(TARGET_TAG_PREFIX)
        return None

    def _target_exists(self, target_id: str | None) -> bool:
        if target_id is None:
            return False
        return (target_id == ROBOT_TARGET_ID and self.robot_spawn is not None) or target_id in self.item_by_prefix

    def select(self, target_id: str) -> None:
        if not self._target_exists(target_id):
            return
        self.selected_target = target_id
        if self.tree is not None:
            current = tuple(str(item) for item in self.tree.selection())
            if current != (target_id,):
                self._selecting_tree = True
                try:
                    self.tree.selection_set(target_id)
                    self.tree.see(target_id)
                finally:
                    self._selecting_tree = False
        self._update_entry_vars(target_id)
        self.redraw()

    def _update_entry_vars(self, target_id: str) -> None:
        if target_id == ROBOT_TARGET_ID and self.robot_spawn is not None and self.robot_xy is not None:
            x, y = self.robot_xy
            self.x_var.set(f"{x:.3f}")
            self.y_var.set(f"{y:.3f}")
            self.z_var.set(f"{self.robot_spawn.z:.3f}")
            self.layer_var.set(self.robot_spawn.layer)
            return
        item = self.item_by_prefix[target_id]
        x, y = self.positions[target_id]
        self.x_var.set(f"{x:.3f}")
        self.y_var.set(f"{y:.3f}")
        self.z_var.set(f"{item.z:.3f}")
        self.layer_var.set("surface" if item.layer == "surface" else "underwater moored")

    def update_selected_from_entries(self) -> bool:
        if self.selected_target is None:
            self.status_var.set("course layout: select an item first")
            return False
        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
        except ValueError:
            self.status_var.set("course layout: invalid X/Y number")
            return False
        self._set_position(self.selected_target, x, y)
        self._update_entry_vars(self.selected_target)
        self._refresh_tree_row(self.selected_target)
        self.redraw()
        return True

    def _set_position(self, target_id: str, x: float, y: float) -> None:
        clamped_x, clamped_y = clamp_xy(x, y)
        if target_id == ROBOT_TARGET_ID:
            self.robot_xy = (clamped_x, clamped_y)
            return
        self.positions[target_id] = (clamped_x, clamped_y)

    def _refresh_tree_row(self, target_id: str) -> None:
        if self.tree is None:
            return
        if target_id == ROBOT_TARGET_ID and self.robot_spawn is not None and self.robot_xy is not None:
            x, y = self.robot_xy
            self.robot_spawn = replace(self.robot_spawn, x=x, y=y)
            self.tree.item(target_id, values=(self.robot_spawn.layer, f"{x:.2f}", f"{y:.2f}", f"{self.robot_spawn.z:.2f}"))
            return
        if target_id not in self.item_by_prefix:
            return
        item = self.item_by_prefix[target_id]
        x, y = self.positions[target_id]
        updated = replace(item, x=x, y=y)
        self.item_by_prefix[target_id] = updated
        self.tree.item(target_id, values=(item.layer, f"{x:.2f}", f"{y:.2f}", f"{item.z:.2f}"))

    def save(self) -> bool:
        if self.selected_target is not None and not self.update_selected_from_entries():
            return False
        try:
            backup = save_buoy_layout(self.scene_path, self.positions, robot_xy=self.robot_xy)
        except Exception as exc:
            self.status_var.set(f"course layout save failed: {exc}")
            self.owner.buoy_layout_status_var.set("course layout: save failed")
            return False
        selected = self.selected_target
        self.reload(select=selected)
        self.status_var.set(f"course layout: saved XML; backup {backup.name}")
        self.owner.buoy_layout_status_var.set("course layout: saved; restart sim to reload")
        return True

    def save_and_reset(self) -> None:
        if not self.save():
            return
        self.owner.buoy_layout_status_var.set("course layout: saved; resetting sim")
        try:
            self.owner._stop_sim_stack()
        except Exception as exc:
            self.status_var.set(f"course layout saved; sim reset failed: {exc}")

    def close(self) -> None:
        if self.win is not None and self.win.winfo_exists():
            self.win.destroy()
        self.owner.buoy_layout_window = None
        self.owner.buoy_layout_canvas = None
        self.owner.buoy_layout_editor = None
        self.win = None
        self.canvas = None
        self.tree = None


def _show_buoy_layout_window(owner: Any) -> None:
    editor = getattr(owner, "buoy_layout_editor", None)
    if editor is None:
        editor = BuoyLayoutEditor(owner)
        owner.buoy_layout_editor = editor
    editor.show()


__all__ = ["BuoyLayoutEditor", "_show_buoy_layout_window"]
