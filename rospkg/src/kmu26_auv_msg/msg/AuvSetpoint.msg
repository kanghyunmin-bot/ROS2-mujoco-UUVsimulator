# AuvSetpoint.msg
float64 x      # [m]
float64 y      # [m]
float64 z      # [m]
float64 yaw    # 예약 필드(현재 무시됨): 헤딩은 현재 위치에서 목표를 향하도록 자동 계산
int32  mode    # 0: odom 절대, 1: 현재 위치+odom축 오프셋, 2: 시작 자세 기준 절대(x=y=z=0)
