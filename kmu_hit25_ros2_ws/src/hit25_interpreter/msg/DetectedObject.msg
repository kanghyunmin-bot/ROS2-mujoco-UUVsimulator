#DetectedObject.msg

# 개별적으로 탐지된 객체 하나의 통합 정보를 담는 메시지

string object_name      # 객체 클래스 이름 (예: "Buoy", "Line", "Qr")
float32 confidence      # 탐지 신뢰도 (0.0 ~ 1.0)

# 공통 정보: 위치와 크기
float32 center_x        # 객체 중심의 x 좌표 (픽셀)
float32 center_y        # 객체 중심의 y 좌표 (픽셀)
float32 width           # 객체의 너비 (픽셀)
float32 height          # 객체의 높이 (픽셀)

# 선택적 정보: 특정 객체 타입에만 해당
float32 slope           # 'Line' 타입일 경우의 기울기 (그 외에는 0 또는 nan)
string qr_data          # 'Qr' 타입일 경우 디코딩된 데이터 (그 외에는 빈 문자열)
