#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec

try:
    from mavros_msgs.msg import State as MavrosState
except Exception:
    MavrosState = None

MAX_POINTS = 100
V_PER_CELL_FULL = 4.2
V_PER_CELL_EMPTY = 3.8


class AUVStateGUI(Node):
    def __init__(self):
        super().__init__('auv_state_gui')
        self.lock = threading.Lock()

        self.mavros_state = None
        self.batt_msg = None

        self.times = []
        self.voltages = []
        self.currents = []

        self.display_pct = 0.0
        self.detected_cells = 0
        self.start_time = time.time()

        self.init_plot()

        self.create_subscription(BatteryState, '/battery', self.batt_cb, 10)
        if MavrosState:
            self.create_subscription(MavrosState, '/mavros/state', self.state_cb, 10)
        else:
            self.get_logger().warn("MAVROS msg not found; /mavros/state will not be displayed.")

    def init_plot(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('AUV State & Battery Monitor')

        gs = gridspec.GridSpec(3, 2, height_ratios=[1, 1, 2])

        self.ax_state = self.fig.add_subplot(gs[0, :])
        self.ax_state.axis('off')
        self.text_state = self.ax_state.text(0.05, 0.5, "Waiting for /mavros/state...",
                                             fontsize=12, va='center', family='monospace')

        self.ax_batt_info = self.fig.add_subplot(gs[1, :])
        self.ax_batt_info.axis('off')
        self.text_batt = self.ax_batt_info.text(0.5, 0.5, "Waiting for /battery...",
                                                fontsize=16, ha='center', va='center', weight='bold', color='blue')

        self.ax_vol = self.fig.add_subplot(gs[2, 0])
        self.ax_cur = self.fig.add_subplot(gs[2, 1])

        self.line_vol, = self.ax_vol.plot([], [], 'b-', label='Voltage (V)')
        self.ax_vol.set_title("Voltage History")
        self.ax_vol.set_ylim(0, 30)
        self.ax_vol.grid(True, linestyle='--')
        self.ax_vol.legend(loc='upper left')

        self.line_cur, = self.ax_cur.plot([], [], 'r-', label='Current (A)')
        self.ax_cur.set_title("Current History")
        self.ax_cur.set_ylim(0, 50)
        self.ax_cur.grid(True, linestyle='--')
        self.ax_cur.legend(loc='upper left')

        plt.tight_layout()

    def state_cb(self, msg):
        with self.lock:
            self.mavros_state = msg

    def batt_cb(self, msg):
        curr_t = time.time() - self.start_time
        curr_v = msg.voltage
        curr_i = abs(msg.current)

        pct = 0.0
        if msg.percentage >= 0:
            pct = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
        else:
            temp_cells = self.detected_cells
            if temp_cells == 0 and curr_v > V_PER_CELL_EMPTY:
                temp_cells = int(round(curr_v / ((V_PER_CELL_FULL + V_PER_CELL_EMPTY) / 2.0)))
                self.detected_cells = temp_cells
            if temp_cells > 0:
                v_full = V_PER_CELL_FULL * temp_cells
                v_empty = V_PER_CELL_EMPTY * temp_cells
                calc = (curr_v - v_empty) / (v_full - v_empty)
                pct = max(0.0, min(1.0, calc)) * 100.0

        with self.lock:
            self.batt_msg = msg
            self.display_pct = pct
            self.times.append(curr_t)
            self.voltages.append(curr_v)
            self.currents.append(curr_i)

            if len(self.times) > MAX_POINTS:
                self.times.pop(0)
                self.voltages.pop(0)
                self.currents.pop(0)

    def update_gui(self, _frame):
        with self.lock:
            if self.mavros_state:
                s = self.mavros_state
                state_str = (
                    f"--- [MAVROS STATE] ---\n"
                    f"Connected: {s.connected}  |  Armed: {s.armed}\n"
                    f"Guided: {getattr(s, 'guided', False)}     |  Manual Input: {getattr(s, 'manual_input', False)}\n"
                    f"Mode: {s.mode}\n"
                    f"System Status: {s.system_status}"
                )
                st_color = 'red' if s.armed else 'black'
                self.text_state.set_text(state_str)
                self.text_state.set_color(st_color)

            if self.batt_msg:
                v = self.batt_msg.voltage
                i = self.batt_msg.current
                p = self.display_pct
                batt_str = f"Voltage: {v:.2f} V   |   Current: {i:.2f} A   |   Battery: {p:.1f} %"
                self.text_batt.set_text(batt_str)
                if p < 20:
                    self.text_batt.set_color('red')
                elif p < 50:
                    self.text_batt.set_color('orange')
                else:
                    self.text_batt.set_color('green')

            if self.times:
                self.line_vol.set_data(self.times, self.voltages)
                self.line_cur.set_data(self.times, self.currents)
                self.ax_vol.set_xlim(min(self.times), max(self.times) + 1)
                self.ax_cur.set_xlim(min(self.times), max(self.times) + 1)
                if max(self.voltages) > self.ax_vol.get_ylim()[1] - 1:
                    self.ax_vol.set_ylim(0, max(self.voltages) + 5)
                if max(self.currents) > self.ax_cur.get_ylim()[1] - 1:
                    self.ax_cur.set_ylim(0, max(self.currents) + 10)

        return self.text_state, self.text_batt, self.line_vol, self.line_cur


def main():
    rclpy.init()
    node = AUVStateGUI()

    def _spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    ani = animation.FuncAnimation(node.fig, node.update_gui, interval=200)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
