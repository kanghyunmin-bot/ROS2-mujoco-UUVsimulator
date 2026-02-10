#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h> // Yaw 제어를 위한 서비스 헤더
#include <tf/transform_datatypes.h> // getYawFromPose를 위해 추가

// non-blocking terminal input
#include <fcntl.h>
#include <termios.h>
#include <unistd.h> // read()

// C++
#include <string>
#include <sstream>
#include <cmath>

// 상태 정의
enum class State {
    AWAITING_ORIGIN, // 원점 대기
    INITIALIZING,
    HOLDING,
    MOVING
};

// --- 전역 변수 ---
State currentState = State::AWAITING_ORIGIN;
mavros_msgs::State current_state_msg;
geometry_msgs::PoseStamped current_pose_msg;
mavros_msgs::PositionTarget target_pose_msg;

// Yaw 제어용 서비스 클라이언트
ros::ServiceClient command_client;

// 원점 저장을 위한 변수
geometry_msgs::PoseStamped origin_pose;
double origin_yaw = 0.0;
bool origin_set = false;
bool pose_received = false; // 첫 포즈 수신 확인용

const double ARRIVAL_THRESHOLD = 0.1; // 도착 인정 범위 (10cm)
int init_counter = 0;
const int INIT_PUBLISH_COUNT = 30; // 3초 (10Hz * 3)

// --- Helper 함수: Pose에서 Yaw(rad) 추출 ---
double getYawFromPose(const geometry_msgs::Pose& pose) {
    tf::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// --- 콜백 함수 ---

// 콜백 함수: MAVROS 상태
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state_msg = *msg;
}

// 콜백 함수: 현재 위치
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 현재 위치는 항상 업데이트
    current_pose_msg = *msg;
    
    // 이 콜백이 최소 한 번 호출되었음을 플래그로 설정
    if (!pose_received) {
        pose_received = true;
    }
}

// 터미널 입력을 비차단 모드로 설정
void setup_nonblocking_stdin() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// --- [수정됨] Helper 함수: Yaw 제어 서비스 호출 ---
void call_yaw_command(double relative_yaw_deg) {
    if (std::abs(relative_yaw_deg) < 0.01) {
        return; // Yaw 변경이 거의 없으면 호출 안 함
    }

    // [수정됨] 회전 방향 설정
    double rotation_direction = 0.0;
    if (relative_yaw_deg > 0) {
        rotation_direction = 1.0; // 시계방향 (CW)
    } else {
        rotation_direction = -1.0; // 반시계방향 (CCW)
    }
    
    // [수정됨] 각도는 절대값으로
    double target_angle_deg = std::abs(relative_yaw_deg);

    ROS_INFO("Requesting relative YAW change: %.2f deg (Direction: %s)", 
             relative_yaw_deg, (rotation_direction > 0) ? "CW" : "CCW");
    
    mavros_msgs::CommandLong srv_msg;
    srv_msg.request.broadcast = false;
    srv_msg.request.command = 115; // MAV_CMD_CONDITION_YAW
    srv_msg.request.confirmation = 0;
    
    // 요청하신 파라미터 설정
    srv_msg.request.param1 = target_angle_deg;    // 목표 yaw 각도 [deg] (절대값)
    srv_msg.request.param2 = 10.0;                // 회전 속도 [deg/s]
    srv_msg.request.param3 = rotation_direction;  // 회전 방향 (1=CW, -1=CCW)
    srv_msg.request.param4 = 1.0;                 // 기준 방식 (1=상대)
    srv_msg.request.param5 = 0.0;                 // 미사용
    srv_msg.request.param6 = 0.0;                 // 미사용
    srv_msg.request.param7 = 0.0;                 // 미사용

    if (command_client.call(srv_msg)) {
        if (!srv_msg.response.success) {
            ROS_WARN("Yaw command failed with result: %d", (int)srv_msg.response.result);
        } else {
            ROS_INFO("Yaw command sent successfully.");
        }
    } else {
        ROS_ERROR("Failed to call MAV_CMD_CONDITION_YAW service");
    }
}


// --- Main 함수 ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "guided_xyz_yaw_node");
    ros::NodeHandle nh;

    // 구독자 (Subscriber)
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, pose_cb);

    // 발행자 (Publisher)
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 10);

    // 서비스 클라이언트 (Service Client)
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("/mavros/set_mode");
    // [추가됨] Yaw 제어를 위한 서비스 클라이언트
    command_client = nh.serviceClient<mavros_msgs::CommandLong>
        ("/mavros/cmd/command");


    // 비차단 터미널 설정
    setup_nonblocking_stdin();

    ROS_INFO("===== GUIDED Mode XYZ + YAW Controller =====");
    ROS_INFO("Waiting for first position message to set origin...");

    // MAVROS에 2Hz 이상으로 발행해야 함
    ros::Rate rate(10.0); // 10Hz

    // PositionTarget 메시지 기본 설정
    target_pose_msg.header.frame_id = "map"; 
    target_pose_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    // Type Mask: 2496 (0b010011100000)
    // 위치(X,Y,Z)만 사용
    // 속도(VX,VY,VZ), 가속도(AX,AY,AZ), Yaw, Yaw Rate 모두 무시
    // *중요*: Yaw를 무시해야 MAV_CMD_CONDITION_YAW와 충돌하지 않음
    target_pose_msg.type_mask = 2496; 
                                
    target_pose_msg.velocity.x = 0.0; // 무시됨
    target_pose_msg.velocity.y = 0.0; // 무시됨
    target_pose_msg.velocity.z = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.x = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.y = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.z = 0.0; // 무시됨
    target_pose_msg.yaw = 0.0;       // 무시됨
    target_pose_msg.yaw_rate = 0.0; // 무시됨

    std::string input_buffer;

    while (ros::ok()) {
        // --- 1. 비차단 터미널 입력 처리 ---
        char buf[1];
        int n = read(STDIN_FILENO, buf, 1);
        if (n > 0) {
            if (buf[0] == '\n') {
                // Enter 키 입력 시 버퍼 처리
                if (currentState == State::HOLDING || currentState == State::MOVING) {
                    
                    // x,y,z,yaw 파싱
                    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
                    char comma; // 콤마를 읽기 위한 변수
                    std::stringstream ss(input_buffer);
                    
                    if (ss >> x >> comma >> y >> comma >> z >> comma >> yaw) {
                        
                        // 1. Yaw 제어 호출 (0이 아닐 경우)
                        call_yaw_command(yaw);

                        // 2. XYZ 위치 제어 (0이 아닐 경우)
                        if (std::abs(x) > 0.01 || std::abs(y) > 0.01 || std::abs(z) > 0.01) {
                            ROS_INFO("New relative position target: x=%.2f, y=%.2f, z=%.2f", x, y, z);

                            // 원점 기준으로 절대 목표 위치 계산
                            target_pose_msg.position.x = origin_pose.pose.position.x + x;
                            target_pose_msg.position.y = origin_pose.pose.position.y + y;
                            target_pose_msg.position.z = origin_pose.pose.position.z + z;

                            currentState = State::MOVING;
                        } else {
                            ROS_INFO("XYZ target unchanged, holding current position.");
                        }

                    } else {
                        ROS_WARN("Invalid format. Use: x,y,z,yaw (e.g., 1,2,-0.5,45)");
                    }
                } else if (currentState == State::AWAITING_ORIGIN) {
                    ROS_INFO("Please wait, still setting origin...");
                }
                
                input_buffer.clear(); // 버퍼 비우기
            } else {
                input_buffer += buf[0]; // 버퍼에 문자 추가
            }
        }

        // --- 2. 상태 머신 (State Machine) ---
        switch (currentState) {
            case State::AWAITING_ORIGIN:
                // 첫 위치 메시지 수신 대기
                if (pose_received && !origin_set) {
                    origin_pose = current_pose_msg;
                    origin_yaw = getYawFromPose(origin_pose.pose);
                    origin_set = true;

                    // 최초 목표 지점 = 원점
                    target_pose_msg.position = origin_pose.pose.position;
                    // target_pose_msg.yaw 는 type_mask로 인해 무시됨

                    ROS_INFO("********************************************");
                    ROS_INFO("Origin Set at [x: %.2f, y: %.2f, z: %.2f, yaw: %.2f rad]", 
                             origin_pose.pose.position.x,
                             origin_pose.pose.position.y,
                             origin_pose.pose.position.z,
                             origin_yaw);
                    ROS_INFO("********************************************");

                    currentState = State::INITIALIZING; // 초기화 상태로 이행
                    init_counter = 0;
                }
                break;

            case State::INITIALIZING:
                // 원점 위치를 3초간 발행 (모드 변경 전 안정성)
                setpoint_pub.publish(target_pose_msg);

                if (init_counter < INIT_PUBLISH_COUNT) {
                    init_counter++;
                } else {
                    // 3초간 발행 후 GUIDED 모드 요청
                    if (current_state_msg.mode != "GUIDED") {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "GUIDED";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                            ROS_INFO("GUIDED mode requested...");
                        } else {
                            ROS_WARN("Failed to request GUIDED mode. Retrying...");
                        }
                    } else {
                        ROS_INFO("GUIDED mode active. Holding at Origin.");
                        ROS_INFO("--------------------------------------------");
                        ROS_INFO("Enter new relative command (x,y,z,yaw):");
                        currentState = State::HOLDING;
                    }
                }
                break;

            case State::HOLDING:
                // 현재 목표 위치(도착했거나 원점)를 계속 발행하여 모드 유지
                setpoint_pub.publish(target_pose_msg);
                break;

            case State::MOVING:
                // 새로운 목표 위치 발행
                setpoint_pub.publish(target_pose_msg);

                // 도착 여부 확인
                if (pose_received) {
                    double dx = target_pose_msg.position.x - current_pose_msg.pose.position.x;
                    double dy = target_pose_msg.position.y - current_pose_msg.pose.position.y;
                    double dz = target_pose_msg.position.z - current_pose_msg.pose.position.z;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (dist < ARRIVAL_THRESHOLD) {
                        ROS_INFO("*************************");
                        ROS_INFO("Arrived at XYZ target (%.2f, %.2f, %.2f)", 
                                 target_pose_msg.position.x, 
                                 target_pose_msg.position.y, 
                                 target_pose_msg.position.z);
                        ROS_INFO("Enter new relative command (x,y,z,yaw):");
                        ROS_INFO("*************************");
                        currentState = State::HOLDING;
                    }
                }
                break;
        }

        // --- 3. 공통 작업 ---
        // 헤더 타임스탬프 업데이트 (HOLDING, MOVING, INITIALIZING 시 발행)
        if (currentState != State::AWAITING_ORIGIN) {
            target_pose_msg.header.stamp = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}