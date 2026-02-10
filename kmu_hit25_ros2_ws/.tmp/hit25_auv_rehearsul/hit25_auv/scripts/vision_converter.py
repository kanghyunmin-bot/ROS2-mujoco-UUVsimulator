# vision_converter.py

import rospy
from hit25_interpreter.msg import VisionData, DetectedObject
from typing import Dict, List, Any
import numpy as np

class VisionConverter:
    """
    /hit25/vision 토픽을 구독하여 VisionData 메시지를 수신하고,
    미션 로직에서 사용하기 쉬운 파이썬 데이터 구조(딕셔너리, 리스트)로 변환하여
    최신 비전 정보를 제공하는 클래스입니다.
    
    메인 제어 파일의 '비전 관련 파일' 역할을 수행합니다.
    """
    
    def __init__(self, topic_name: str = '/hit25/vision'):
        """
        초기화: ROS 서브스크라이버를 설정하고 내부 데이터 버퍼를 준비합니다.
        :param topic_name: VisionData 메시지를 구독할 토픽 이름 (기본값: '/hit25/vision')
        """
        rospy.loginfo(f"[VisionConverter] Initializing subscriber to {topic_name}")
        self._latest_data: Dict[str, Any] = {}
        self._latest_objects: List[DetectedObject] = []
        
        # ROS 서브스크라이버 설정
        self.sub = rospy.Subscriber(topic_name, VisionData, self._vision_callback)
        
    def _vision_callback(self, msg: VisionData):
        """
        VisionData 메시지가 수신될 때마다 호출되는 콜백 함수입니다.
        수신된 데이터를 내부 버퍼에 저장하고 파싱합니다.
        """
        # 헤더 및 화면 크기 정보 저장
        self._latest_data['header'] = msg.header
        self._latest_data['screen_width'] = msg.screen_width
        self._latest_data['screen_height'] = msg.screen_height
        
        # 탐지된 객체 리스트를 파이썬 리스트에 직접 저장 (원시 메시지 객체)
        self._latest_objects = msg.detected_objects
        
        # (선택적) 디버깅 및 데이터 유효성 검증
        # rospy.logdebug_throttle(1.0, f"Vision Data Updated. Detected: {len(self._latest_objects)} objects.")

    def get_latest_vision_data(self) -> List[DetectedObject]:
        """
        가장 최근에 탐지된 객체 목록(DetectedObject 리스트)을 반환합니다.
        """
        return self._latest_objects

    def find_objects_by_name(self, object_name: str) -> List[DetectedObject]:
        """
        특정 이름(예: 'Buoy', 'Line', 'Qr')을 가진 객체들만 필터링하여 반환합니다.

        :param object_name: 찾고자 하는 객체의 이름 (대소문자 구분)
        :return: 필터링된 DetectedObject 리스트
        """
        if not self._latest_objects:
            return []
            
        return [obj for obj in self._latest_objects if obj.object_name == object_name]

    def get_qr_data(self) -> str:
        """
        탐지된 QR 코드가 있다면 첫 번째 QR 코드의 디코딩된 데이터를 반환합니다.
        """
        qr_objects = self.find_objects_by_name('Qr')
        if qr_objects:
            # 첫 번째 탐지된 QR 객체의 데이터를 반환
            return qr_objects[0].qr_data
        return ""


# ======================================================================
# 주석: VisionConverter 사용 예시
# ======================================================================
"""
# 1. 초기화 (메인 제어 파일 내부)
rospy.init_node('mission_node')
vision_sensor = VisionConverter('/hit25/vision')

# 2. 메인 루프에서 데이터 사용
# while not rospy.is_shutdown():
    
    # 부표 객체 탐지
    buoys = vision_sensor.find_objects_by_name('Buoy')
    if buoys:
        # 가장 신뢰도 높은 부표의 픽셀 중심 좌표 사용
        main_buoy = max(buoys, key=lambda b: b.confidence)
        center_x = main_buoy.center_x
        
        # (Yawing PID를 위한 목표 각 계산 로직은 메인 제어 파일에 구현)
        # target_yaw_offset = (center_x - screen_width / 2) * K_vision

    # QR 코드 데이터 확인
    address = vision_sensor.get_qr_data()
    if address:
        rospy.loginfo(f"QR Address Detected: {address}")

    # rospy.sleep(0.02) # 50Hz
"""