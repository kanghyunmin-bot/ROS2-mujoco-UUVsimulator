#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
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
    HOLDING
};

// --- 전역 변수 ---
State currentState = State::AWAITING_ORIGIN;
mavros_msgs::State current_state_msg;
geometry_msgs::PoseStamped current_pose_msg; // [추가됨]
mavros_msgs::PositionTarget target_pose_msg; // 목표 Yaw

// [추가됨] 원점 저장을 위한 변수
geometry_msgs::PoseStamped origin_pose;
double origin_yaw = 0.0;
bool origin_set = false;
bool pose_received = false; 

int init_counter = 0;
const int INIT_PUBLISH_COUNT = 30; // 3초 (10Hz * 3)

// [추가됨] Helper 함수: Pose에서 Yaw(rad) 추출
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
    // [수정됨] 현재 위치는 항상 업데이트
    current_pose_msg = *msg; 
    
    if (!pose_received) {
        pose_received = true;
    }
}

// 터미널 입력을 비차단 모드로 설정
void setup_nonblocking_stdin() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// --- Main 함수 ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "guided_setpoint_yaw_node"); 
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

    // 비차단 터미널 설정
    setup_nonblocking_stdin();

    ROS_INFO("===== GUIDED Mode Yaw (Rate) Controller ====="); 
    ROS_INFO("Waiting for first position message to set origin..."); // [수정됨]

    ros::Rate rate(10.0); // 10Hz

    // PositionTarget 메시지 기본 설정 (초기화용)
    target_pose_msg.header.frame_id = "map"; 
    
    // [수정됨] 초기화 시에는 안정적인 FRAME_LOCAL_NED (1) 사용
    target_pose_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; 
    
    // [수정됨] 초기화 시에는 안정적인 2552 (위치+Yaw) 사용
    target_pose_msg.type_mask = 2496;
                                   
    target_pose_msg.velocity.x = 0.0; // 무시됨
    target_pose_msg.velocity.y = 0.0; // 무시됨
    target_pose_msg.velocity.z = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.x = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.y = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.z = 0.0; // 무시됨
    
    // (AWAITING_ORIGIN에서 현재 값으로 채워질 것임)
    target_pose_msg.position.x = 0.0; 
    target_pose_msg.position.y = 0.0; 
    target_pose_msg.position.z = 0.0; 
    target_pose_msg.yaw = 0.0;        
    target_pose_msg.yaw_rate = 0.0; // 무시됨

    std::string input_buffer;

    while (ros::ok()) {
        // --- 1. 비차단 터미널 입력 처리 ---
        char buf[1];
        int n = read(STDIN_FILENO, buf, 1);
        if (n > 0) {
            if (buf[0] == '\n') {
                if (currentState == State::HOLDING) { 
                    
                    double yaw_rel_cmd = 0.0;
                    std::stringstream ss(input_buffer);
                    
                    if (ss >> yaw_rel_cmd) {
                        ROS_INFO("New relative yaw rate command: %.2f rad/s (approx)", yaw_rel_cmd);
                        
                        // [수정됨] HOLDING 상태에서는 Frame 9, Mask 2559 사용
                        target_pose_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
                        target_pose_msg.type_mask = 2559; // Yaw-only mask
                        target_pose_msg.yaw = yaw_rel_cmd;
                        // (Position 필드는 0이지만, 2559 마스크에 의해 무시됨)
                        target_pose_msg.position.x = 0.0;
                        target_pose_msg.position.y = 0.0;
                        target_pose_msg.position.z = 0.0;
                        
                    } else {
                        ROS_WARN("Invalid format. Use: yaw (e.g., 0.5)");
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
                // [수정됨] "안정적인 코드"의 초기화 로직
                if (pose_received && !origin_set) {
                    origin_pose = current_pose_msg;
                    origin_yaw = getYawFromPose(origin_pose.pose);
                    origin_set = true;

                    // 최초 목표 지점 = 현재 위치 (원점)
                    target_pose_msg.position = origin_pose.pose.position;
                    target_pose_msg.yaw = origin_yaw; 

                    ROS_INFO("********************************************");
                    ROS_INFO("Origin Set at [x: %.2f, y: %.2f, z: %.2f, yaw: %.2f rad]", 
                             origin_pose.pose.position.x,
                             origin_pose.pose.position.y,
                             origin_pose.pose.position.z,
                             origin_yaw);
                    ROS_INFO("********************************************");

                    currentState = State::INITIALIZING; 
                    init_counter = 0;
                }
                break;

            case State::INITIALIZING:
                // [수정됨] 원점 위치/Yaw를 3초간 발행 (frame=1, mask=2552)
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
                        ROS_INFO("GUIDED mode active. Holding Origin Position and Yaw.");
                        ROS_INFO("--------------------------------------------");
                        ROS_INFO("Enter new relative yaw (rad, e.g., 0.5 or 0.0):"); // [수정됨]
                        
                        // [수정됨] HOLDING 상태 진입 시, frame=9, mask=2559, yaw=0.0 (정지)로 전환
                        target_pose_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
                        target_pose_msg.type_mask = 2559; // Yaw-only
                        target_pose_msg.position.x = 0.0; // 무시됨
                        target_pose_msg.position.y = 0.0; // 무시됨
                        target_pose_msg.position.z = 0.0; // 무시됨
                        target_pose_msg.yaw = 0.0; // Yaw Rate 0.0 (Hold)
                        
                        currentState = State::HOLDING;
                    }
                }
                break;

            case State::HOLDING:
                // 현재 설정된 yaw (rate) 값을 10Hz로 계속 발행
                // (frame=9, mask=2559)
                setpoint_pub.publish(target_pose_msg);
                break;
        }

        // --- 3. 공통 작업 ---
        // 헤더 타임스탬프 업데이트
        if (currentState != State::AWAITING_ORIGIN) {
            target_pose_msg.header.stamp = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}