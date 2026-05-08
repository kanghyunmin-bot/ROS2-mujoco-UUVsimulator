"""Manual control, drawing, and telemetry update methods for the GUI."""

from __future__ import annotations

import datetime as _dt

from .config import *
from .helpers import *
from .models import ControlCommands, RcReplaySample
from .node import UuvGuiNode
from .ros_tools import *
from .runtime import *
from .widgets import VirtualJoystick

class ControlDisplayMixin:
    def _zero_controls(self) -> None:
        self._center_rc_sticks()

    def _center_rc_sticks(self) -> None:
        self.rc_forward_var.set(0.0)
        self.rc_lateral_var.set(0.0)
        self.rc_heave_var.set(0.0)
        self.rc_yaw_var.set(0.0)

    def _release_rc_override(self) -> None:
        self.rc_override_enabled.set(False)
        self._center_rc_sticks()
        self.node.publish_rc_release()

    def _toggle_control_details(self) -> None:
        show = not self.control_details_visible.get()
        self.control_details_visible.set(show)
        if show:
            self.control_details_frame.grid()
            self.control_details_button.config(text="Hide control details")
        else:
            self.control_details_frame.grid_remove()
            self.control_details_button.config(text="Show control details")

    def _toggle_vehicle_details(self) -> None:
        show = not self.vehicle_details_visible.get()
        self.vehicle_details_visible.set(show)
        if show:
            self.vehicle_details_frame.grid()
            self.vehicle_details_button.config(text="Details v")
        else:
            self.vehicle_details_frame.grid_remove()
            self.vehicle_details_button.config(text="Details >")

    def _toggle_telemetry_panel(self) -> None:
        show = not self.telemetry_visible.get()
        self.telemetry_visible.set(show)
        if self.main_container is None or self.telemetry_panel is None or self.control_panel is None:
            return

        if show:
            self.root.minsize(*WINDOW_MINSIZE)
            self.telemetry_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
            self.control_panel.grid_configure(row=0, column=1, columnspan=1, sticky="nsew")
            self.main_container.columnconfigure(0, weight=3)
            self.main_container.columnconfigure(1, weight=2)
            if self.root.winfo_width() < WINDOW_MINSIZE[0]:
                self.root.geometry(f"{WINDOW_MINSIZE[0]}x{max(self.root.winfo_height(), WINDOW_MINSIZE[1])}")
            self.telemetry_toggle_button.config(text="Hide telemetry")
        else:
            self.telemetry_panel.grid_remove()
            self.control_panel.grid_configure(row=0, column=0, columnspan=2, sticky="nsew")
            self.main_container.columnconfigure(0, weight=1)
            self.main_container.columnconfigure(1, weight=0)
            self.root.minsize(*TELEMETRY_HIDDEN_MINSIZE)
            self.root.geometry(
                f"{TELEMETRY_HIDDEN_WIDTH}x{max(self.root.winfo_height(), TELEMETRY_HIDDEN_MINSIZE[1])}"
            )
            self.telemetry_toggle_button.config(text="Show telemetry")
    def _on_rc_override_toggle(self) -> None:
        if self.rc_override_enabled.get() and self._rc_replay_running():
            self._stop_rc_replay()
        if self.rc_override_enabled.get():
            pass
        else:
            self.node.publish_rc_release()

    def _draw_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        canvas = self.attitude_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 100)
        cx = width / 2.0
        cy = height / 2.0

        pitch_offset = clamp(pitch_deg, -45.0, 45.0) * 2.2
        roll_rad = math.radians(roll_deg)
        extent = max(width, height) * 1.8
        half = extent / 2.0
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)

        def rot(x: float, y: float) -> tuple[float, float]:
            return (cx + x * cos_r - y * sin_r, cy + x * sin_r + y * cos_r)

        sky = [
            rot(-half, -half - pitch_offset),
            rot(half, -half - pitch_offset),
            rot(half, -pitch_offset),
            rot(-half, -pitch_offset),
        ]
        ground = [
            rot(-half, -pitch_offset),
            rot(half, -pitch_offset),
            rot(half, half - pitch_offset),
            rot(-half, half - pitch_offset),
        ]
        canvas.create_polygon(*sum(([x, y] for x, y in sky), []), fill="#1d4ed8", outline="")
        canvas.create_polygon(*sum(([x, y] for x, y in ground), []), fill="#854d0e", outline="")

        left = rot(-half, -pitch_offset)
        right = rot(half, -pitch_offset)
        canvas.create_line(left[0], left[1], right[0], right[1], fill="white", width=3)

        for step in range(-30, 35, 10):
            if step == 0:
                continue
            y_line = -pitch_offset - step * 2.2
            span = 60 if step % 20 == 0 else 30
            p1 = rot(-span, y_line)
            p2 = rot(span, y_line)
            canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill="#e2e8f0", width=2)

        canvas.create_line(cx - 70, cy, cx - 15, cy, fill="#f8fafc", width=4)
        canvas.create_line(cx + 15, cy, cx + 70, cy, fill="#f8fafc", width=4)
        canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, outline="#f8fafc", width=2)
        canvas.create_line(cx, cy - 18, cx, cy + 18, fill="#f8fafc", width=2)

        canvas.create_text(
            12,
            12,
            anchor="nw",
            fill="#f8fafc",
            font=("TkDefaultFont", 12, "bold"),
            text=f"ROLL {roll_deg:+05.1f}  PITCH {pitch_deg:+05.1f}  YAW {yaw_deg:+06.1f}",
        )

    def _draw_depth(self, depth_m: float, source: str) -> None:
        canvas = self.depth_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 140)
        height = max(canvas.winfo_height(), 82)
        pad = 12
        value_valid = math.isfinite(depth_m)
        display_depth = depth_m if value_valid else 0.0
        clamped_depth = clamp(display_depth, 0.0, MAX_DEPTH_DISPLAY_M)
        ratio = clamped_depth / max(MAX_DEPTH_DISPLAY_M, 1e-6)
        value_text = f"{depth_m:.2f} m" if value_valid else "n/a"
        source_text = source if len(source) <= 28 else f"{source[:25]}..."

        canvas.create_rectangle(0, 0, width, height, fill="#0f172a", outline="")
        canvas.create_text(pad, 11, anchor="nw", fill="#94a3b8", font=("TkDefaultFont", 9, "bold"), text="DEPTH")
        canvas.create_text(
            pad,
            31,
            anchor="w",
            fill="#e0f2fe" if value_valid else "#64748b",
            font=("TkDefaultFont", 20, "bold"),
            text=value_text,
        )
        canvas.create_text(width - pad, 15, anchor="ne", fill="#64748b", font=("TkDefaultFont", 8), text=source_text)

        bar_x0 = pad
        bar_x1 = width - pad
        bar_y0 = height - 26
        bar_y1 = height - 15
        canvas.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1, fill="#1e293b", outline="#334155")
        if value_valid:
            canvas.create_rectangle(
                bar_x0 + 1,
                bar_y0 + 1,
                bar_x0 + 1 + (bar_x1 - bar_x0 - 2) * ratio,
                bar_y1 - 1,
                fill="#38bdf8",
                outline="",
            )
        canvas.create_text(bar_x0, height - 6, anchor="sw", fill="#94a3b8", font=("TkDefaultFont", 8), text="0")
        canvas.create_text(
            bar_x1,
            height - 6,
            anchor="se",
            fill="#94a3b8",
            font=("TkDefaultFont", 8),
            text=f"{MAX_DEPTH_DISPLAY_M:g} m",
        )

    def _update_events(self, events: Deque[str]) -> None:
        top = events[0] if events else ""
        if top == self._last_event_top:
            return
        self._last_event_top = top
        self.event_list.delete(0, tk.END)
        for item in events:
            self.event_list.insert(tk.END, item)

    def _read_control_commands(self) -> ControlCommands:
        rc_forward, rc_lateral, rc_heave, rc_yaw = gui_rc_to_override_axes(
            forward=self.rc_forward_var.get(),
            lateral=self.rc_lateral_var.get(),
            heave=self.rc_heave_var.get(),
            yaw=self.rc_yaw_var.get(),
        )
        return ControlCommands(
            velocity_forward=0.0,
            velocity_lateral=0.0,
            velocity_heave=0.0,
            velocity_yaw=0.0,
            rc_forward=rc_forward,
            rc_lateral=rc_lateral,
            rc_heave=rc_heave,
            rc_yaw=rc_yaw,
        )

    def _publish_active_controls(self, commands: ControlCommands) -> None:
        rc_active = self.rc_override_enabled.get()

        if rc_active:
            self.node.publish_rc_override(
                yaw=commands.rc_yaw,
                heave=commands.rc_heave,
                forward=commands.rc_forward,
                lateral=commands.rc_lateral,
            )
        elif self._rc_override_prev:
            self.node.publish_rc_release()
        self._rc_override_prev = rc_active

    def _update_rc_feedback_bars(self, channels: list[int]) -> None:
        for idx, channel in enumerate(channels[:RC_VISIBLE_CHANNEL_COUNT]):
            value = int(channel)
            self._rc_bars[idx]["value"] = clamp(value - 1100, 0, 800) if value > 0 else 0
            self._rc_labels[idx].config(text=str(value))

    def _update_ui(self) -> None:
        if self._closed or not self.root.winfo_exists():
            return
        snap = self.node.snapshot()
        self.node.probe_backend()
        now = time.monotonic()
        if now - self._last_vehicle_info_wall > 2.0:
            self.node.request_vehicle_info()
            self._last_vehicle_info_wall = now

        backend_label = self.node.backend_label()
        rc_mapping_summary = self.node.rc_mapping_summary()
        mode_display = snap.vehicle_mode or snap.mode
        state_text = (
            f"connected={snap.connected}  armed={snap.armed}  guided={snap.guided}  "
            f"manual_input={snap.manual_input}  backend={backend_label}"
        )
        self.status_var.set(state_text)
        self.mode_var.set(f"mode: {mode_display}  (raw={snap.mode}, id={snap.mode_id}, state={snap.system_status})")

        depth_summary = f"{snap.depth_m:.2f} m" if math.isfinite(snap.depth_m) else "n/a"
        vehicle_state = "connected" if snap.connected else "disconnected"
        arm_state = "armed" if snap.armed else "disarmed"
        self.vehicle_summary_var.set(
            f"{vehicle_state} | {arm_state} | {mode_display} | depth {depth_summary}"
        )

        batt_pct = snap.battery_percent * 100.0 if math.isfinite(snap.battery_percent) else math.nan
        batt_text = (
            f"battery: {snap.battery_voltage:.2f} V, {snap.battery_current:.2f} A, "
            f"{batt_pct:.0f}%"
            if math.isfinite(snap.battery_voltage)
            else "battery: n/a"
        )
        self.battery_var.set(batt_text)

        px, py, pz = snap.position_xyz
        self.pose_var.set(
            f"pose: x={px:+.2f}  y={py:+.2f}  z={pz:+.2f}"
            if math.isfinite(px)
            else "pose: n/a"
        )
        vx, vy, vz = snap.velocity_xyz
        self.vel_var.set(
            f"velocity: x={vx:+.2f}  y={vy:+.2f}  z={vz:+.2f}  src={snap.velocity_source}"
            if math.isfinite(vx)
            else f"velocity: n/a  src={snap.velocity_source}"
        )
        vel_summary = f"vel ({vx:+.2f}, {vy:+.2f}, {vz:+.2f}) m/s" if math.isfinite(vx) else "vel n/a"
        wx, wy, wz = snap.ang_vel_xyz
        self.imu_var.set(
            f"imu: roll={snap.roll_deg:+.1f}  pitch={snap.pitch_deg:+.1f}  yaw={snap.yaw_deg:+.1f}  "
            f"gyro=({wx:+.2f}, {wy:+.2f}, {wz:+.2f})"
        )
        self.motion_summary_var.set(
            f"{vel_summary} | rpy ({snap.roll_deg:+.1f}, {snap.pitch_deg:+.1f}, {snap.yaw_deg:+.1f})"
        )
        autopilot_text = snap.autopilot_name
        if not autopilot_text:
            if self.node.vehicle_info_supported():
                autopilot_text = "pending vehicle_info_get"
            else:
                autopilot_text = "vehicle_info_get unavailable"
        self.autopilot_var.set(
            f"autopilot: {autopilot_text}  rc-map={rc_mapping_summary}"
        )

        if math.isfinite(snap.depth_m):
            self.depth_target_var.set(f"depth: {snap.depth_m:.2f} m")
        else:
            self.depth_target_var.set("depth: n/a")
        self.depth_source_var.set(f"depth source: {snap.depth_source}")

        self.age_var.set(
            "age: "
            f"state={format_age(snap.state_age_s)}, "
            f"imu={format_age(snap.imu_age_s)}, "
            f"pose={format_age(snap.pose_age_s)}, "
            f"depth={format_age(snap.depth_age_s)}, "
            f"rc={format_age(snap.rc_age_s)}"
        )
        ping360_age = format_age(snap.ping360_age_s)
        self.ping360_summary_var.set(f"{snap.ping360_summary}  age={ping360_age}")

        commands = self._read_control_commands()
        if self.rc_override_enabled.get():
            control_mode = "RC override"
        elif self._rc_replay_running():
            control_mode = "RC replay"
        else:
            control_mode = "idle"
        self.control_summary_var.set(
            f"control: {control_mode}  details={'shown' if self.control_details_visible.get() else 'hidden'}"
        )
        self.control_var.set(
            f"rc setpoint: fwd={commands.rc_forward:+.2f}  lat={commands.rc_lateral:+.2f}  "
            f"heave={commands.rc_heave:+.2f}  yaw={commands.rc_yaw:+.2f}"
        )
        self.rc_override_var.set(
            "rc override: "
            f"{'on' if self.rc_override_enabled.get() else 'off'}  "
            f"{rc_mapping_summary}  "
            f"heave={axis_to_pwm(commands.rc_heave)}  yaw={axis_to_pwm(commands.rc_yaw)}  "
            f"forward={axis_to_pwm(commands.rc_forward)}  lateral={axis_to_pwm(commands.rc_lateral)}  "
            f"feedback={snap.rc_feedback_source}"
        )
        self._refresh_ros2_buttons()

        self._publish_active_controls(commands)

        self._draw_attitude(snap.roll_deg, snap.pitch_deg, snap.yaw_deg)
        self._draw_depth(snap.depth_m, snap.depth_source)
        self._update_events(snap.events)

        self._update_rc_feedback_bars(snap.rc_out)

        self._schedule_update()
