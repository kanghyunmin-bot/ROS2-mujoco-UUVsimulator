# 비전 시스템이 분석한 전체 정보를 담는 메시지
# 모든 탐지된 객체는 detected_objects 리스트 안에 통합되어 관리됩니다.

std_msgs/Header header

# 분석에 사용된 이미지 원본 크기
int32 screen_width
int32 screen_height

# 탐지된 모든 객체 목록
DetectedObject[] detected_objects
