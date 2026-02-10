# trajectory_calculator.py

import numpy as np
import math
from scipy.interpolate import CubicSpline, CubicHermiteSpline
from typing import List, Tuple, Union

# 계산 결과를 담는 구조체 (경로의 한 점)
Position = Tuple[float, float] # (x, y) 튜플

class TrajectoryCalculator:
    """
    주어진 N개의 Waypoint(시작점 + 부표 오프셋점들)을 기반으로
    경로를 생성하는 클래스입니다.
    
    [수정 사항]
    1. 부표 통과 시, 부표가 위치한 방향(중심선 기준)의 바깥쪽으로 오프셋을 적용하여 S자 경로를 만듭니다.
    2. 부표 위치(Waypoint)가 경로의 정확한 정점(Peak/Trough)이 되도록 Hermite Spline을 사용해 기울기를 0으로 고정합니다.
    """
    
    # 기본 오프셋 값 (0.2m -> 필요에 따라 늘려서 S자 굴곡을 키울 수 있음)
    DEFAULT_OFFSET_M = 0.05
    
    def __init__(self, offset_m: float = DEFAULT_OFFSET_M):
        """
        TrajectoryCalculator를 초기화합니다.
        
        :param offset_m: 부표 위치로부터의 Y-축 오프셋 거리 (m).
        """
        self.offset_m = abs(offset_m) # 오프셋 거리는 항상 양수로 저장

    def _apply_buoy_offset(self, 
                           buoy_pos: Position,
                           prev_pos: Position
                           ) -> Position:
        """
        [수정됨] 부표 위치를 기준으로 '바깥쪽(Outward)' 통과 지점을 계산합니다.
        
        1. 부표가 Y > 0 (왼쪽/위)에 있으면 -> 더 위쪽(+Offset)으로 통과
        2. 부표가 Y < 0 (오른쪽/아래)에 있으면 -> 더 아래쪽(-Offset)으로 통과
        3. 부표가 Y ≈ 0 (중앙)에 있으면 -> 로봇(이전 점)의 반대편으로 통과 (Slalom)
        
        :param buoy_pos: 부표의 (x, y) 위치
        :param prev_pos: 로봇의 현재 위치 또는 직전 Waypoint 위치 (x, y)
        :return: 오프셋이 적용된 통과 지점 (x, y)
        """
        x_buoy, y_buoy = buoy_pos
        _, y_prev = prev_pos
        
        y_target = 0.0
        
        # 노이즈를 고려한 중앙 판별 마진
        CENTER_MARGIN = 0.05 

        # Case A: 부표가 중심선보다 확실히 위(Y > 0)에 있음 -> 더 위로(+Offset)
        if y_buoy > CENTER_MARGIN:
            y_target = y_buoy + self.offset_m
            
        # Case B: 부표가 중심선보다 확실히 아래(Y < 0)에 있음 -> 더 아래로(-Offset)
        elif y_buoy < -CENTER_MARGIN:
            y_target = y_buoy - self.offset_m
            
        # Case C: 부표가 정중앙(Y ≈ 0)에 있음 -> 이전 위치를 보고 지그재그 회피
        else:
            # 이전 위치가 위였으면 아래로, 아래였으면 위로 통과
            if y_prev >= 0:
                y_target = y_buoy - self.offset_m # 아래로 통과
            else:
                y_target = y_buoy + self.offset_m # 위로 통과
            
        return (x_buoy, y_target)

    def generate_path(self, 
                      current_pos: Position, 
                      buoy_positions: List[Position],
                      num_path_points: int = 100
                      ) -> Union[List[Position], None]:
        """
        로봇 현재 위치와 N개의 부표 위치를 기반으로 경로를 생성합니다.
        
        [핵심 변경]
        - Y축 경로 생성 시 CubicHermiteSpline을 사용하여 모든 웨이포인트에서의 기울기(dy/dt)를 0으로 설정합니다.
        - 이를 통해 부표의 X 위치가 정확히 경로의 극점(Peak/Trough)이 되도록 보장합니다.

        :param current_pos: 로봇의 현재 (x, y) 위치
        :param buoy_positions: N개의 부표 위치 리스트 (최소 3개 권장)
        :param num_path_points: 생성할 경로 점의 개수
        :return: (x, y) 튜플로 이루어진 경로 Waypoint 리스트. 실패 시 None.
        """
        
        if len(buoy_positions) < 3:
            print(f"[TrajectoryCalculator] 부표 위치는 최소 3개가 필요합니다. (현재: {len(buoy_positions)})")
            return None
        
        # 1. Waypoint 리스트 구성
        waypoints: List[Position] = []
        
        # W0: 시작점
        waypoints.append(current_pos)
        prev_pos = current_pos
        
        # W1~WN: 부표 오프셋 통과 지점 계산
        for pos in buoy_positions:
            target_pos = self._apply_buoy_offset(pos, prev_pos)
            waypoints.append(target_pos)
            prev_pos = target_pos
            
        # 2. Waypoint를 x, y 배열로 분리
        x_points = np.array([p[0] for p in waypoints])
        y_points = np.array([p[1] for p in waypoints])
        
        # 3. 경로 파라미터 t 계산 (누적 거리)
        dx = np.diff(x_points)
        dy = np.diff(y_points)
        segment_lengths = np.sqrt(dx**2 + dy**2)
        t_points = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        
        # 4. 보간 함수 생성
        try:
            # X축: 부드러운 진행을 위해 일반 CubicSpline 사용
            cs_x = CubicSpline(t_points, x_points)
            
            # Y축: 극점 일치를 위해 CubicHermiteSpline 사용
            # 모든 웨이포인트에서 기울기(dy/dt)를 0으로 설정 -> 수평 접선 강제
            dydt = np.zeros_like(y_points) 
            cs_y = CubicHermiteSpline(t_points, y_points, dydt)
            
        except Exception as e:
            print(f"[TrajectoryCalculator] 스플라인 생성 실패: {e}")
            return None

        # 5. 경로 점 생성 (보간)
        t_new = np.linspace(0.0, t_points[-1], num_path_points)
        path_x = cs_x(t_new)
        path_y = cs_y(t_new)
        
        # 6. 반환
        return list(zip(path_x, path_y))

# ======================================================================
# 주석: TrajectoryCalculator 사용 예시
# ======================================================================
"""
import rospy
# from auv_setpoint_sender import AUVSetpointSender # (필요 시 임포트)

# 1. 초기화 (메인 제어 파일 내부)
rospy.init_node('mission_node')
# 0.2m 오프셋으로 초기화 (또는 기본값 사용)
path_generator = TrajectoryCalculator(offset_m=0.2) 

# 2. 데이터 준비
# 로봇 현재 위치 (x, y)
current_pos_auv = (0.0, 0.0) 

# 부표 예상 위치 리스트 (3개 이상)
# 예: 감지된 부표 + 하드코딩 부표 + 가상 부표(Dummy)
buoy_positions_list = [
    (2.0, 0.3),   # 부표 1: y=0.3 (위) -> 위로 통과(+Offset) -> Target y=0.5
    (4.0, -0.3),  # 부표 2: y=-0.3 (아래) -> 아래로 통과(-Offset) -> Target y=-0.5
    (6.0, 0.0),   # 부표 3: y=0.0 -> 지그재그
    (7.5, 0.5)    # 부표 4 (Dummy)
]

# 3. 경로 생성
path = path_generator.generate_path(current_pos_auv, buoy_positions_list, num_path_points=50)

if path:
    print(f"생성된 경로 점 개수: {len(path)}")
    print(f"경로 시작점: {path[0]}")
    print(f"경로 끝점: {path[-1]}")
    
    # 4. (미션 로직) 경로 추종
    # for i, (wp_x, wp_y) in enumerate(path):
    #     # Yaw 계산 ...
    #     # auv_comm.send_setpoint(...)
else:
    print("경로 생성에 실패했습니다.")
"""