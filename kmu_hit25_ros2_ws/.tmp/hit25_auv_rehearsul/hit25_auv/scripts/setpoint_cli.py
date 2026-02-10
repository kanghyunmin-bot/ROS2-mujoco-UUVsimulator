#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
setpoint_cli.py

이 노드는 AUV 컨트롤러 테스트를 위해 터미널에서 수동으로
/auv_setpoint 메시지를 발행하는 CLI(Command Line Interface) 도구입니다.

- 사용자로부터 x, y, z, yaw, mode를 입력받습니다.
- 확인 후, 지정된 시간(pub_duration) 동안 지정된 주기(pub_rate)로
  동일한 setpoint 메시지를 반복적으로 발행합니다.
- 이는 컨트롤러가 첫 메시지를 놓치는 경우를 방지하기 위한 테스트용 동작입니다.
"""

import rospy
import math
import sys
import threading

# TODO: 실제 프로젝트의 메시지 타입으로 변경해야 합니다.
try:
    from hit25_auv.msg import AuvSetpoint
except ImportError:
    # 이 노드는 AuvSetpoint 메시지 없이는 동작할 수 없으므로,
    # rospy.init_node() 호출 전에 치명적 오류로 처리합니다.
    print(
        "ERROR: 'hit25_auv.msg.AuvSetpoint' 메시지를 import할 수 없습니다."
        "ROS 패키지 빌드 및 'source devel/setup.bash'를 확인하세요.",
        file=sys.stderr
    )
    sys.exit(1)


class SetpointCLI(object):
    """
    /auv_setpoint 발행을 위한 CLI 인터페이스 클래스
    """
    def __init__(self):
        """
        노드 초기화, 파라미터 로드, Publisher 생성을 수행합니다.
        """
        rospy.loginfo("AUV Setpoint CLI 노드 초기화 중...")

        # --- 파라미터 로드 ---
        self.load_parameters()

        # --- Publisher 초기화 ---
        self.setpoint_pub = rospy.Publisher(
            "/auv_setpoint", AuvSetpoint, queue_size=10
        )

        rospy.loginfo("Setpoint Publisher 생성 완료. (/auv_setpoint)")
        rospy.loginfo(f"좌표계 기준 힌트: {self.frame_hint}")


    def load_parameters(self):
        """
        ROS 파라미터 서버로부터 설정을 로드합니다.
        """
        # 반복 발행 설정
        self.pub_rate_hz = rospy.get_param("~pub_rate", 5.0)
        self.pub_duration_s = rospy.get_param("~pub_duration", 1.0)
        
        # UI/안내용
        self.default_mode = rospy.get_param("~default_mode", 0)
        self.frame_hint = rospy.get_param("~frame_hint", "odom")
        
        if self.pub_rate_hz <= 0:
            rospy.logwarn("~pub_rate가 0보다 작거나 같으므로 1.0 Hz로 강제 조정합니다.")
            self.pub_rate_hz = 1.0


    def run(self):
        """
        메인 CLI 루프를 실행합니다.
        사용자 입력을 받고 setpoint 발행을 트리거합니다.
        """
        try:
            while not rospy.is_shutdown():
                # 1. 메인 메뉴 표시
                self.print_main_menu()
                choice = input("선택 입력 (a/r/q): ").strip().lower()

                # 2. 입력 처리
                if choice == 'q':
                    break  # 메인 루프 탈출

                elif choice in ('a', 'r'):
                    mode = 0 if choice == 'a' else 1
                    mode_str = "ABSOLUTE (odom 기준)" if mode == 0 else "RELATIVE (현재 기준)"
                    
                    rospy.loginfo(f"{mode_str} 모드 선택됨 (mode = {mode})")

                    # 3. 값 입력 받기 (x, y, z, yaw)
                    try:
                        x = self.get_float_input("x [m] 입력 (엔터로 스킵시 0.0): ")
                        y = self.get_float_input("y [m] 입력 (엔터로 스킵시 0.0): ")
                        z = self.get_float_input("z [m] 입력 (엔터로 스킵시 0.0): ")
                        yaw_deg = self.get_float_input("yaw [deg] 입력 (엔터로 스킵시 0.0): ")
                    except (rospy.ROSInterruptException, KeyboardInterrupt):
                        # get_float_input 내부에서 Ctrl+C가 눌린 경우
                        rospy.loginfo("\n입력 취소. 메인 메뉴로 돌아갑니다.")
                        continue

                    yaw_rad = math.radians(yaw_deg)

                    # 4. 발행 전 확인
                    if self.confirm_setpoint(mode, mode_str, x, y, z, yaw_deg, yaw_rad):
                        # 5. 확인 시 반복 발행
                        self.publish_setpoint_repeatedly(mode, x, y, z, yaw_rad)
                    else:
                        rospy.loginfo("발행이 취소되었습니다. 메인 메뉴로 돌아갑니다.")

                else:
                    rospy.logwarn("잘못된 입력입니다. 'a', 'r', 'q' 중 하나를 입력하세요.")

        except (rospy.ROSInterruptException, KeyboardInterrupt):
            # 메인 루프(input 대기) 중 Ctrl+C
            rospy.loginfo("\nCtrl+C 감지. 종료 중...")
        
        rospy.loginfo("AUV Setpoint CLI 종료")

    def print_main_menu(self):
        """
        CLI 메인 메뉴를 터미널에 출력합니다.
        """
        print("\n" + "="*30)
        print("     ===== AUV Setpoint CLI =====")
        print(f"     (좌표계 기준 힌트: {self.frame_hint})")
        print("="*30)
        print(" (a) 절대 좌표 모드 (mode = 0)")
        print(" (r) 상대 좌표 모드 (mode = 1)")
        print(" (q) 종료")

    def get_float_input(self, prompt):
        """
        사용자로부터 float 입력을 받습니다.
        빈 문자열은 0.0으로 처리하고, 잘못된 입력은 다시 묻습니다.
        """
        while not rospy.is_shutdown():
            try:
                val_str = input(prompt).strip()
                if val_str == "":
                    return 0.0
                return float(val_str)
            except ValueError:
                rospy.logwarn("잘못된 숫자 형식입니다. 다시 입력하거나 엔터를 눌러 0.0을 사용하세요.")
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                # Ctrl+C가 눌리면 예외를 다시 발생시켜 상위 루프가 처리하도록 함
                raise

    def confirm_setpoint(self, mode, mode_str, x, y, z, yaw_deg, yaw_rad):
        """
        발행할 setpoint를 요약해서 보여주고 사용자 확인(y/n)을 받습니다.
        """
        print("\n" + "--- 입력된 Setpoint ---")
        print(f"  mode:    {mode} ({mode_str})")
        print(f"  x:       {x:.3f} [m]")
        print(f"  y:       {y:.3f} [m]")
        print(f"  z:       {z:.3f} [m]")
        print(f"  yaw_deg: {yaw_deg:.2f} [deg]")
        print(f"  yaw_rad: {yaw_rad:.4f} [rad]")
        print("-"*(len("--- 입력된 Setpoint ---")))
        
        while not rospy.is_shutdown():
            try:
                confirm = input("발행할까요? (y/n): ").strip().lower()
                if confirm == 'y':
                    return True
                if confirm == 'n':
                    return False
                print(" 'y' 또는 'n' 을 입력하세요.")
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                raise # 상위로 전파

    def publish_setpoint_repeatedly(self, mode, x, y, z, yaw_rad):
        """
        AuvSetpoint 메시지를 생성하여 지정된 시간/주기 동안 반복 발행합니다.
        """
        msg = AuvSetpoint()
        msg.mode = mode
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw_rad

        rate = rospy.Rate(self.pub_rate_hz)
        end_time = rospy.Time.now() + rospy.Duration(self.pub_duration_s)

        rospy.loginfo(
            f"{self.pub_duration_s}초 동안 {self.pub_rate_hz}Hz 주기로 setpoint 발행 시작..."
        )
        
        publish_count = 0
        try:
            while rospy.Time.now() < end_time and not rospy.is_shutdown():
                self.setpoint_pub.publish(msg)
                publish_count += 1
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("발행 중 노드 종료 감지. 발행 중단.")
        
        rospy.loginfo(f"Setpoint 발행 완료 (총 {publish_count}회 발행).")


if __name__ == "__main__":
    """
    노드 실행 메인 함수
    """
    try:
        rospy.init_node("auv_setpoint_cli", anonymous=False)
        cli = SetpointCLI()
        cli.run()

    except rospy.ROSInterruptException:
        # init_node 또는 클래스 생성 중 Ctrl+C
        rospy.loginfo("AUV Setpoint CLI 시작 중 종료됨.")
    except Exception as e:
        rospy.logfatal(f"치명적인 예외 발생: {e}")
        import traceback
        traceback.print_exc()