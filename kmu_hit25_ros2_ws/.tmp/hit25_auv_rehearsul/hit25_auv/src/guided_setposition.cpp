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
    HOLDING,
    MOVING
};

// --- 전역 변수 ---
State currentState = State::AWAITING_ORIGIN;
mavros_msgs::State current_state_msg;
geometry_msgs::PoseStamped current_pose_msg;
mavros_msgs::PositionTarget target_pose_msg;

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

// --- Main 함수 ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "guided_setposition_node");
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

    ROS_INFO("===== GUIDED Mode Position Controller =====");
    ROS_INFO("Waiting for first position message to set origin...");

    // MAVROS에 2Hz 이상으로 발행해야 함
    ros::Rate rate(10.0); // 10Hz

    // PositionTarget 메시지 기본 설정
    target_pose_msg.header.frame_id = "map"; 
    target_pose_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    // [수정됨] Type Mask: 3576 (0b110111111000)
    // 위치(X,Y,Z)만 사용
    // 속도(VX,VY,VZ), 가속도(AX,AY,AZ), Yaw, Yaw Rate 모두 무시
    target_pose_msg.type_mask = 2496; 
                                   
    target_pose_msg.velocity.x = 0.0; // 무시됨
    target_pose_msg.velocity.y = 0.0; // 무시됨
    target_pose_msg.velocity.z = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.x = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.y = 0.0; // 무시됨
    target_pose_msg.acceleration_or_force.z = 0.0; // 무시됨
    target_pose_msg.yaw = 0.0;        // 무시됨 (원점 Yaw로 고정될 것임)
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
                    
                    // [수정됨] x,y,z 파싱
                    double x = 0.0, y = 0.0, z = 0.0;
                    char comma; // 콤마를 읽기 위한 변수
                    std::stringstream ss(input_buffer);
                    
                    if (ss >> x >> comma >> y >> comma >> z) {
                        ROS_INFO("New relative target: x=%.2f, y=%.2f, z=%.2f", x, y, z);

                        // [수정됨] 원점 기준으로 절대 목표 위치 계산
                        target_pose_msg.position.x = origin_pose.pose.position.x + x;
                        target_pose_msg.position.y = origin_pose.pose.position.y + y;
                        target_pose_msg.position.z = origin_pose.pose.position.z + z;
                        // target_pose_msg.yaw 는 원점 값으로 고정 (변경 없음)
                        // target_pose_msg.yaw_rate 는 0.0 (변경 없음)

                        currentState = State::MOVING;

                    } else {
                        ROS_WARN("Invalid format. Use: x,y,z (e.g., 1,2,-0.5)");
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
                    // [수정됨] Yaw를 원점 Yaw로 고정
                    target_pose_msg.yaw = origin_yaw; 

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
                        ROS_INFO("Enter new relative position (x,y,z):"); // [수정됨]
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
                        ROS_INFO("Arrived at target (%.2f, %.2f, %.2f)", 
                                 target_pose_msg.position.x, 
                                 target_pose_msg.position.y, 
                                 target_pose_msg.position.z);
                        ROS_INFO("Enter new relative position (x,y,z):"); // [수정됨]
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