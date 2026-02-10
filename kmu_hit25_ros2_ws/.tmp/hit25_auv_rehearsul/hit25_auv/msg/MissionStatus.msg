# MissionStatus.msg

# 상태 상수 정의 (코드 가독성을 위해 사용)
int8 READY = 0      # 미션 시작 전 (대기)
int8 RUNNING = 1    # 미션 진행 중
int8 COMPLETED = 2  # 미션 완료

# 실제 상태 값 데이터
int8 status