#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker

# 데이터 저장 리스트
times = []
voltages = []
currents = []
battery_percentage = 0.0
max_points = 100

V_PER_CELL_FULL = 4.2
V_PER_CELL_EMPTY = 3.8

data_lock = threading.Lock()

fig = plt.figure()
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)

voltage_text = fig.text(0.35, 0.96, "Voltage: N/A V", fontsize=12, color="blue", ha='center', va='top')
current_text = fig.text(0.65, 0.96, "Current: N/A A", fontsize=12, color="red", ha='center', va='top')
percent_text = fig.text(0.35, 0.01, f"Battery: {battery_percentage:.1f}%", fontsize=12, color="green", ha='center')
cell_count_text = fig.text(0.65, 0.01, "Cells: N/A", fontsize=12, color="blue", ha='center')

detected_cell_count = 0


def update_graph(_):
    with data_lock:
        local_times = list(times)
        local_voltages = list(voltages)
        local_currents = list(currents)
        local_percentage = battery_percentage
        local_cell_count = detected_cell_count

    ax1.clear()
    ax2.clear()

    ax1.plot(local_times, local_voltages, label="Voltage (V)", color='blue')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Voltage (V)")
    ax1.legend(loc='upper left')
    ax1.yaxis.set_major_locator(ticker.MultipleLocator(5))
    ax1.yaxis.set_minor_locator(ticker.MultipleLocator(1))
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax1.grid(True, which='major', linestyle='-', linewidth=0.7)
    ax1.set_ylim(0, 30)

    ax2.plot(local_times, local_currents, label="Current (A)", color='red')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Current (A)")
    ax2.legend(loc='upper left')
    ax2.yaxis.set_major_locator(ticker.MultipleLocator(10))
    ax2.yaxis.set_minor_locator(ticker.MultipleLocator(1))
    ax2.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax2.grid(True, which='major', linestyle='-', linewidth=0.7)
    ax2.set_ylim(0, 80)

    percent_text.set_text(f"Battery: {local_percentage:.1f}%")
    cell_text = f"Cells: {local_cell_count}S" if local_cell_count > 0 else "Cells: N/A"
    cell_count_text.set_text(cell_text)

    if local_voltages:
        voltage_text.set_text(f"Voltage: {local_voltages[-1]:.2f} V")
        current_text.set_text(f"Current: {local_currents[-1]:.2f} A")
    else:
        voltage_text.set_text("Voltage: N/A V")
        current_text.set_text("Current: N/A A")

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])


class BatteryVisualizer(Node):
    def __init__(self):
        super().__init__('battery_visualizer')
        self.create_subscription(BatteryState, '/battery', self.battery_callback, 10)

    def battery_callback(self, msg: BatteryState):
        global times, voltages, currents, battery_percentage, detected_cell_count

        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        current_v = msg.voltage
        current_i = abs(msg.current)

        temp_detected_cell_count = detected_cell_count

        if temp_detected_cell_count == 0:
            if len(msg.cell_voltage) > 0:
                temp_detected_cell_count = len(msg.cell_voltage)
                self.get_logger().info(
                    f"Battery visualizer: Detected {temp_detected_cell_count}S battery via cell_voltage list."
                )
            elif current_v > V_PER_CELL_EMPTY:
                avg_cell_v = (V_PER_CELL_FULL + V_PER_CELL_EMPTY) / 2.0
                estimated_cells = int(round(current_v / avg_cell_v))
                if estimated_cells > 0:
                    temp_detected_cell_count = estimated_cells
                    self.get_logger().info(
                        f"Battery visualizer: Assuming {temp_detected_cell_count}S battery based on voltage {current_v:.2f}V."
                    )

        if temp_detected_cell_count > 0:
            v_full = V_PER_CELL_FULL * temp_detected_cell_count
            v_empty = V_PER_CELL_EMPTY * temp_detected_cell_count
            if (v_full - v_empty) > 0:
                percentage_calc = (current_v - v_empty) / (v_full - v_empty)
            else:
                percentage_calc = 0.0
            percentage_calc = max(0.0, min(1.0, percentage_calc))
            local_battery_percentage = percentage_calc * 100
        else:
            local_battery_percentage = 0.0

        with data_lock:
            times.append(stamp)
            voltages.append(current_v)
            currents.append(current_i)
            battery_percentage = local_battery_percentage
            detected_cell_count = temp_detected_cell_count

            if len(times) > max_points:
                times.pop(0)
                voltages.pop(0)
                currents.pop(0)


def main():
    rclpy.init()
    node = BatteryVisualizer()

    def _spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    ani = animation.FuncAnimation(fig, update_graph, interval=111)
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
