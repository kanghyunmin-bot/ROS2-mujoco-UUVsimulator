#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
profilers.py

제어용 궤적 생성(Motion Profiling) 클래스 모음.
Controller 등 다른 노드에서 import하여 사용합니다.
"""

import math
import numpy as np

class TrapezoidalScalarProfiler:
    """
    1차원 값(예: Yaw)에 대한 사다리꼴 속도 프로파일 생성기
    - 가속도 제한(Acceleration Limit) 적용
    - 360도 회전(Angle Wrapping) 지원
    """
    def __init__(self, max_vel, max_accel, is_angle=False):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.is_angle = is_angle
        
        self.curr_pos = 0.0
        self.curr_vel = 0.0

    def reset(self, current_pos, current_vel=0.0):
        """초기 상태 재설정"""
        self.curr_pos = current_pos
        self.curr_vel = current_vel

    def update(self, target_pos, dt):
        """
        다음 스텝의 목표 위치와 속도를 반환
        return: (next_pos, next_vel)
        """
        # 1. 거리 계산 (Angle 모드일 경우 최단 경로)
        if self.is_angle:
            dist = self._wrap_angle(target_pos - self.curr_pos)
        else:
            dist = target_pos - self.curr_pos

        # 2. 안전 속도 계산 (v^2 = 2ax: 제동 거리 확보)
        if self.max_accel > 0:
            safe_vel = math.sqrt(2.0 * self.max_accel * abs(dist))
        else:
            safe_vel = self.max_vel

        # 3. 목표 속도 결정 (최대 속도 vs 안전 속도)
        des_vel_mag = min(self.max_vel, safe_vel)
        des_vel = des_vel_mag * np.sign(dist)

        # 4. 가속도 제한 (Velocity Ramp)
        vel_diff = des_vel - self.curr_vel
        max_change = self.max_accel * dt

        if abs(vel_diff) > max_change:
            self.curr_vel += max_change * np.sign(vel_diff)
        else:
            self.curr_vel = des_vel

        # 5. 위치 적분
        self.curr_pos += self.curr_vel * dt
        
        if self.is_angle:
            self.curr_pos = self._wrap_angle(self.curr_pos)

        return self.curr_pos, self.curr_vel

    @staticmethod
    def _wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


class TrapezoidalVectorProfiler:
    """
    2차원 벡터(예: X, Y 위치)에 대한 사다리꼴 속도 프로파일 생성기
    - 두 축의 비율을 유지하여 직선(Linear) 이동을 보장
    """
    def __init__(self, max_vel, max_accel):
        self.max_vel = max_vel
        self.max_accel = max_accel
        
        self.curr_pos = np.array([0.0, 0.0])
        self.curr_vel = np.array([0.0, 0.0]) # [vx, vy]

    def reset(self, current_pos_xy):
        """초기 상태 재설정 input: [x, y]"""
        self.curr_pos = np.array(current_pos_xy, dtype=float)
        self.curr_vel = np.array([0.0, 0.0])

    def update(self, target_pos_xy, dt):
        """
        return: (next_pos_xy, next_vel_xy)
        """
        target = np.array(target_pos_xy, dtype=float)
        
        # 1. 거리 벡터 및 거리 계산
        diff = target - self.curr_pos
        dist = np.linalg.norm(diff)

        # 2. 안전 속도 (Magnitude)
        if self.max_accel > 0:
            safe_speed = math.sqrt(2.0 * self.max_accel * dist)
        else:
            safe_speed = self.max_vel
        
        target_speed = min(self.max_vel, safe_speed)

        # 3. 목표 속도 벡터 분해 (방향 유지)
        if dist > 0.001:
            des_vel = (diff / dist) * target_speed
        else:
            des_vel = np.array([0.0, 0.0])

        # 4. 가속도 제한 (등방성 가속도 적용)
        vel_diff = des_vel - self.curr_vel
        vel_diff_norm = np.linalg.norm(vel_diff)
        max_change = self.max_accel * dt

        if vel_diff_norm > max_change:
            self.curr_vel += (vel_diff / vel_diff_norm) * max_change
        else:
            self.curr_vel = des_vel

        # 5. 위치 적분
        self.curr_pos += self.curr_vel * dt
        
        return self.curr_pos, self.curr_vel