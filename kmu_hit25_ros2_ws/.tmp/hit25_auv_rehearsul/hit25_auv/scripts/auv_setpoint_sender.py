#!/usr/bin/env python
# -*- coding: utf-8 -*-

# auv_setpoint_sender.py (AuvSetpoint.msg 타입 사용)

import rospy
from typing import Union, Dict

# [수정] hit25.msg 대신, 사용자님의 패키지 이름인 hit25_auv 내부에 메시지가 있다고 가정합니다.
# (실제 실행 시 이 메시지 파일을 해당 패키지에 추가하고 빌드해야 합니다.)
try:
    from hit25_auv.msg import AuvSetpoint
except ImportError:
    # AuvSetpoint 메시지가 없을 경우, 임시로 클래스를 정의하여 ImportError를 방지합니다.
    # 단, 이 경우 ROS 토픽 발행은 실제 AuvSetpoint 메시지 타입에 의존합니다.
    rospy.logwarn("AuvSetpoint.msg를 찾을 수 없습니다. Mock 클래스를 사용합니다. 빌드 상태를 확인하세요!")
    class AuvSetpoint:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0
        yaw: float = 0.0
        mode: int = 0
    
class AUVSetpointSender:
    """
    미션 로직으로부터 받은 목표 위치/자세 명령을 /auv_setpoint 토픽으로
    ROS 시스템에 발행(Publish)하는 순수 Output 클래스입니다.
    AuvSetpoint.msg 타입을 사용합니다.
    """
    
    def __init__(self, setpoint_topic: str = '/auv_setpoint'):
        
        rospy.loginfo(f"[AUVSetpointSender] Initializing publisher to {setpoint_topic} (Type: AuvSetpoint).")
        
        # 1. 퍼블리셔 초기화 (AuvSetpoint 메시지 타입 사용)
        self.pub = rospy.Publisher(setpoint_topic, AuvSetpoint, queue_size=10)
        self.setpoint_msg = AuvSetpoint() 
        
    def send_setpoint(self, 
                      x: float, 
                      y: float, 
                      z: float, 
                      yaw: float, 
                      is_relative: bool = True):
        """ 
        목표 위치와 자세 명령을 /auv_setpoint 토픽으로 발행합니다. 
        float64 타입의 x, y, z, yaw와 int32 타입의 mode를 설정합니다.
        """
        
        mode = 1 if is_relative else 0 # mode는 int32 타입에 맞춰 0 또는 1 (int)
        
        # 메시지 필드에 값 할당
        self.setpoint_msg.x = x
        self.setpoint_msg.y = y
        self.setpoint_msg.z = z
        self.setpoint_msg.yaw = yaw
        self.setpoint_msg.mode = mode
        
        self.pub.publish(self.setpoint_msg)