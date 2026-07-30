# DVL의 velocity와 position_local 데이터를 통합한 메시지
std_msgs/Header header # ROS 수신 시간

# Velocity 정보 (DVL.msg 기반)
geometry_msgs/Vector3 velocity     # vx, vy, vz [m/s]
uint32 time_ms                   # DVL "time" 필드 [ms] (dt 계산용)
float32 fom                      # Figure of Merit
bool velocity_valid              # DVL 유효성 플래그
float64 altitude                 # (참고용)

# Attitude 정보 (DVLPose.msg 기반, 라디안 변환)
float64 roll_rad
float64 pitch_rad
float64 yaw_rad

# Position 정보 (참고용)
float64 x
float64 y
float64 z