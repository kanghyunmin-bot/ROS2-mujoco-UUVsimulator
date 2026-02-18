#!/usr/bin/env python3
# object_detector.py

import logging
import math
from typing import List, Tuple, Union

import numpy as np

# 계산 결과를 담는 구조체 (namedtuple 등을 사용하지 않고 간단한 딕셔너리로 반환)
Position = Tuple[float, float] # (x, y) 튜플

class ObjectDetector:
    """
    미션 로직으로부터 받은 3개의 관측점(AUV 위치/자세) 데이터를 기반으로
    부표 또는 게이트의 예상 위치를 기하학적으로 계산하는 클래스입니다.

    - 관측점: AUV의 (x, y) 위치와 Yaw 각(rad)
    - 계산 방법: 3개의 관측선을 이용한 교점 또는 교차원 중심 추정.
    """
    
    def __init__(self, max_intersection_diameter: float = 0.4):
        """
        ObjectDetector를 초기화합니다.
        
        :param max_intersection_diameter: 3개의 2직선 교점이 만드는 삼각형에
                                          내접하는 원의 최대 허용 직경 (m).
                                          이 값보다 크면 '위치 추정 실패'로 간주됩니다.
        """
        self.MAX_DIAMETER = max_intersection_diameter
        self._logger = logging.getLogger("ObjectDetector")
        
    def _calculate_intersection(self, 
                                x1: float, y1: float, yaw1: float, 
                                x2: float, y2: float, yaw2: float
                                ) -> Union[Position, None]:
        """
        두 관측점 (AUV 위치)에서 객체 방향으로 뻗어나가는 두 직선의 교점을 계산합니다.
        객체의 위치는 AUV 위치를 지나는 기울기 tan(yaw)인 직선 위에 있다고 가정합니다.
        
        :return: 교점의 (x, y) 좌표 튜플, 평행하여 교점이 없으면 None
        """
        # 직선 방정식: (y - y1) = m * (x - x1)  =>  Ax + By = C
        # A = m, B = -1, C = m*x1 - y1
        
        # Yaw를 이용한 기울기(m) 계산
        m1 = math.tan(yaw1)
        m2 = math.tan(yaw2)
        
        # 두 직선의 계수
        A1, B1, C1 = m1, -1.0, m1 * x1 - y1
        A2, B2, C2 = m2, -1.0, m2 * x2 - y2
        
        # 행렬식 계산 (평행 여부 확인)
        determinant = A1 * B2 - A2 * B1
        
        # 두 직선이 평행하거나 거의 평행함
        if abs(determinant) < 1e-6:
            return None 

        # 교점 계산 (크래머의 규칙)
        px = (C1 * B2 - C2 * B1) / determinant
        py = (A1 * C2 - A2 * C1) / determinant
        
        return (px, py)

    def _calculate_circle_center_and_radius(self, 
                                            p1: Position, 
                                            p2: Position, 
                                            p3: Position
                                            ) -> Union[Tuple[Position, float], None]:
        """
        3개의 교점(p1, p2, p3)을 지나는 외접원의 중심과 반지름을 계산합니다.
        (이 원의 직경이 추정 오차의 척도가 됩니다.)
        
        :return: (중심 좌표 (x, y), 반지름 R) 튜플, 세 점이 일직선 상에 있으면 None
        """
        # 교점 좌표
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        
        # 외접원 중심 O(px, py) 계산
        # 행렬식 D = x1(y2-y3) + x2(y3-y1) + x3(y1-y2)
        D = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
        
        # 세 점이 일직선 상에 있으면 D=0
        if abs(D) < 1e-6:
            return None 

        # N = x^2 + y^2
        N1 = x1**2 + y1**2
        N2 = x2**2 + y2**2
        N3 = x3**2 + y3**2

        # 중심 x 좌표
        px = (N1 * (y2 - y3) + N2 * (y3 - y1) + N3 * (y1 - y2)) / (2 * D)
        # 중심 y 좌표
        py = (N1 * (x3 - x2) + N2 * (x1 - x3) + N3 * (x2 - x1)) / (2 * D)

        center = (px, py)
        
        # 반지름 R = 중심과 임의의 한 교점(p1) 사이의 거리
        radius = math.sqrt((px - x1)**2 + (py - y1)**2)

        return (center, radius)


    def estimate_position(self, 
                          data: List[Tuple[float, float, float]]
                          ) -> Union[Position, None]:
        """
        3개의 관측 데이터 리스트를 받아 예상 객체 위치를 계산합니다.
        
        :param data: 3개의 관측 데이터 리스트. 각 요소는 (x, y, yaw) 튜플입니다.
                     예: [(x1, y1, yaw1), (x2, y2, yaw2), (x3, y3, yaw3)]
        :return: 추정된 객체 위치의 (x, y) 튜플. 추정 실패 시 None.
        """
        if len(data) != 3:
            self._logger.error("[ObjectDetector] 입력 데이터는 정확히 3개의 관측점이어야 합니다.")
            return None

        (x1, y1, yaw1) = data[0]
        (x2, y2, yaw2) = data[1]
        (x3, y3, yaw3) = data[2]

        # 1. 3개의 2직선 교점 계산 (P_12, P_23, P_31)
        p12 = self._calculate_intersection(x1, y1, yaw1, x2, y2, yaw2)
        p23 = self._calculate_intersection(x2, y2, yaw2, x3, y3, yaw3)
        p31 = self._calculate_intersection(x3, y3, yaw3, x1, y1, yaw1)
        
        intersections = [p12, p23, p31]
        
        # 평행한 직선이 있어 교점이 2개 이하인 경우 (거의 발생하지 않음)
        if intersections.count(None) >= 2:
            self._logger.warning("[ObjectDetector] 추정 실패: 두 쌍 이상의 관측선이 평행합니다.")
            return None
        
        # None 값을 제외한 실제 교점만 필터링
        valid_points = [p for p in intersections if p is not None]
        
        # 교점이 3개가 아닌 경우 (2개만 유효한 경우)
        if len(valid_points) < 3:
             self._logger.warning("[ObjectDetector] 추정 실패: 유효한 교점이 3개 미만입니다.")
             return None

        # 2. 3개 교점을 지나는 외접원의 중심과 반지름 계산
        circle_result = self._calculate_circle_center_and_radius(valid_points[0], valid_points[1], valid_points[2])

        if circle_result is None:
            self._logger.warning("[ObjectDetector] 추정 실패: 3개 교점이 일직선상에 있습니다 (Degenerate case).")
            # 이 경우 3개의 교점의 단순 평균을 반환할 수도 있으나, 여기서는 실패 처리
            return None
            
        center, radius = circle_result
        diameter = radius * 2.0
        
        # 3. 교차원 직경 기반 유효성 검사
        if diameter <= self.MAX_DIAMETER:
            # 허용 오차 이내: 외접원의 중심을 예상 위치로 반환
            self._logger.info(
                "[ObjectDetector] 추정 성공: 직경 %.3fm < 허용 %.3fm. 중심 반환.",
                diameter, self.MAX_DIAMETER
            )
            return center
        else:
            # 허용 오차 초과: 추정 실패 (관측 데이터 불일치)
            self._logger.warning(
                "[ObjectDetector] 추정 실패: 직경 %.3fm > 허용 %.3fm. 데이터 불일치.",
                diameter, self.MAX_DIAMETER
            )
            return None

# ======================================================================
# 주석: ObjectDetector 사용 예시
# ======================================================================
"""
# 1. 초기화 (메인 제어 파일 내부)
# 오차 허용 직경을 0.5m로 설정하여 인스턴스화
detector = ObjectDetector(max_intersection_diameter=0.5)

# 2. 미션 로직에서 3개 관측 데이터 수집
# (x, y, yaw) 튜플 리스트
# AUV 위치 (1, 0, 0rad)에서 0.5rad 각도로 부표 관측
# AUV 위치 (2, 1, 1.0rad)에서 1.0rad 각도로 부표 관측
# AUV 위치 (0, 1, -1.0rad)에서 -1.0rad 각도로 부표 관측
observation_data = [
    (1.0, 0.0, 0.5), 
    (2.0, 1.0, 1.0), 
    (0.0, 1.0, -1.0) 
]

# 3. 예상 위치 계산
estimated_buoy_pos = detector.estimate_position(observation_data)

if estimated_buoy_pos:
    print(f"예상 객체 위치 (x, y): {estimated_buoy_pos}")
    # 메인 제어 파일은 이 위치로 이동하는 Trajectory 생성/명령 전송
else:
    print("객체 위치 추정 실패. 데이터 재수집 필요.")
"""
