# 음향 신호로부터 추출된 핵심 특징 정보를 담는 메시지

std_msgs/Header header             # ROS 표준 헤더 (타임스탬프, 프레임 ID)

float32 frequency         # 현재 감지된 가장 우세한 주파수 (Hz)
float32 decibels          # 해당 주파수의 신호 세기 (dB)
float32 confidence        # 이 신호가 유효한 목표 비콘일 신뢰도 (0.0 ~ 1.0)
float32 target_frequency  # 감지된 목표 주파수 (일치하는 경우, 아니면 0.0)
