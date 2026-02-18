#!/usr/bin/env python3
import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from hit25_auv.msg import AuvSetpoint
    _HAVE_MSG = True
except Exception:
    AuvSetpoint = None
    _HAVE_MSG = False


class SetpointCLI(Node):
    def __init__(self):
        super().__init__("auv_setpoint_cli")

        if not _HAVE_MSG:
            self.get_logger().fatal("AuvSetpoint.msg not available.")
            raise RuntimeError("AuvSetpoint.msg not available")

        self.pub_rate_hz = self.declare_parameter("pub_rate", 5.0).value
        self.pub_duration_s = self.declare_parameter("pub_duration", 1.0).value
        self.default_mode = self.declare_parameter("default_mode", 0).value
        self.frame_hint = self.declare_parameter("frame_hint", "odom").value

        if self.pub_rate_hz <= 0:
            self.get_logger().warn("~pub_rate <= 0; forcing to 1.0 Hz")
            self.pub_rate_hz = 1.0

        self.setpoint_pub = self.create_publisher(AuvSetpoint, "/auv_setpoint", 10)
        self.get_logger().info("Setpoint Publisher created. (/auv_setpoint)")
        self.get_logger().info(f"좌표계 기준 힌트: {self.frame_hint}")

    def run(self):
        try:
            while rclpy.ok():
                self.print_main_menu()
                choice = input("선택 입력 (a/r/q): ").strip().lower()

                if choice == 'q':
                    break

                if choice in ('a', 'r'):
                    mode = 0 if choice == 'a' else 1
                    mode_str = "ABSOLUTE (odom 기준)" if mode == 0 else "RELATIVE (현재 기준)"
                    self.get_logger().info(f"{mode_str} 모드 선택됨 (mode = {mode})")

                    try:
                        x = self.get_float_input("x [m] 입력 (엔터로 스킵시 0.0): ")
                        y = self.get_float_input("y [m] 입력 (엔터로 스킵시 0.0): ")
                        z = self.get_float_input("z [m] 입력 (엔터로 스킵시 0.0): ")
                        yaw_deg = self.get_float_input("yaw [deg] 입력 (엔터로 스킵시 0.0): ")
                    except KeyboardInterrupt:
                        self.get_logger().info("입력 취소. 메인 메뉴로 돌아갑니다.")
                        continue

                    yaw_rad = math.radians(yaw_deg)

                    if self.confirm_setpoint(mode, mode_str, x, y, z, yaw_deg, yaw_rad):
                        self.publish_setpoint_repeatedly(mode, x, y, z, yaw_rad)
                    else:
                        self.get_logger().info("발행이 취소되었습니다. 메인 메뉴로 돌아갑니다.")
                else:
                    self.get_logger().warn("잘못된 입력입니다. 'a', 'r', 'q' 중 하나를 입력하세요.")
        except KeyboardInterrupt:
            self.get_logger().info("Ctrl+C 감지. 종료 중...")

        self.get_logger().info("AUV Setpoint CLI 종료")

    def print_main_menu(self):
        print("\n" + "=" * 30)
        print("     ===== AUV Setpoint CLI =====")
        print(f"     (좌표계 기준 힌트: {self.frame_hint})")
        print("=" * 30)
        print(" (a) 절대 좌표 모드 (mode = 0)")
        print(" (r) 상대 좌표 모드 (mode = 1)")
        print(" (q) 종료")

    def get_float_input(self, prompt):
        while rclpy.ok():
            val_str = input(prompt).strip()
            if val_str == "":
                return 0.0
            try:
                return float(val_str)
            except ValueError:
                self.get_logger().warn("잘못된 숫자 형식입니다. 다시 입력하거나 엔터를 눌러 0.0을 사용하세요.")

    def confirm_setpoint(self, mode, mode_str, x, y, z, yaw_deg, yaw_rad):
        print("\n" + "--- 입력된 Setpoint ---")
        print(f"  mode:    {mode} ({mode_str})")
        print(f"  x:       {x:.3f} [m]")
        print(f"  y:       {y:.3f} [m]")
        print(f"  z:       {z:.3f} [m]")
        print(f"  yaw_deg: {yaw_deg:.2f} [deg]")
        print(f"  yaw_rad: {yaw_rad:.4f} [rad]")
        print("-" * (len("--- 입력된 Setpoint ---")))

        while rclpy.ok():
            confirm = input("발행할까요? (y/n): ").strip().lower()
            if confirm == 'y':
                return True
            if confirm == 'n':
                return False
            print(" 'y' 또는 'n' 을 입력하세요.")

    def publish_setpoint_repeatedly(self, mode, x, y, z, yaw_rad):
        msg = AuvSetpoint()
        msg.mode = mode
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw_rad

        rate = self.create_rate(self.pub_rate_hz)
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=self.pub_duration_s)

        self.get_logger().info(
            f"{self.pub_duration_s}초 동안 {self.pub_rate_hz}Hz 주기로 setpoint 발행 시작..."
        )
        publish_count = 0
        while self.get_clock().now() < end_time and rclpy.ok():
            self.setpoint_pub.publish(msg)
            publish_count += 1
            rate.sleep()

        self.get_logger().info(f"Setpoint 발행 완료 (총 {publish_count}회 발행).")


def main():
    rclpy.init()
    node = None
    try:
        node = SetpointCLI()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"치명적인 예외 발생: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
