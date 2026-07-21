# YawControlDebug.msg

# Yaw 제어 디버깅용
float32 target_yaw   # 목표 각도 [rad]
float32 current_yaw  # 현재 각도 [rad]
float32 error_yaw    # 오차 (Target - Current) [rad]

# 위치 이동 디버깅용
float32 target_x
float32 current_x
float32 error_distance # 목표점까지 남은 거리 [m]