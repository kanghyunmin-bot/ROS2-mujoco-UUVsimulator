#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <sstream>
#include <cmath>

enum class State {
    AWAITING_ORIGIN, // 원점 대기
    INITIALIZING,
    HOLDING,
    MOVING
};

class GuidedControlNode : public rclcpp::Node {
public:
    GuidedControlNode()
    : Node("guided_xyz_yaw_node") {
        setpoint_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", rclcpp::QoS(10)
        );
        state_sub = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", rclcpp::QoS(10),
            std::bind(&GuidedControlNode::state_cb, this, std::placeholders::_1)
        );
        local_pos_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::QoS(10),
            std::bind(&GuidedControlNode::pose_cb, this, std::placeholders::_1)
        );
        set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        command_client = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

        setup_nonblocking_stdin();

        RCLCPP_INFO(this->get_logger(), "===== GUIDED Mode XYZ + YAW Controller =====");
        RCLCPP_INFO(this->get_logger(), "Waiting for first position message to set origin...");

        

        target_pose_msg.header.frame_id = "map";
        target_pose_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        target_pose_msg.type_mask = 2496;

        target_pose_msg.position.x = 0.0;
        target_pose_msg.position.y = 0.0;
        target_pose_msg.position.z = 0.0;
        target_pose_msg.acceleration_or_force.x = 0.0;
        target_pose_msg.acceleration_or_force.y = 0.0;
        target_pose_msg.acceleration_or_force.z = 0.0;
        target_pose_msg.yaw = 0.0;
        target_pose_msg.yaw_rate = 0.0;

        
    }

    void loop() { 
        char buf[1];
        int n = read(STDIN_FILENO, buf, 1);
        if (n > 0) {
            if (buf[0] == '\n') {
                if (currentState == State::HOLDING || currentState == State::MOVING) {

                    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
                    char comma;
                    std::stringstream ss(input_buffer);

                    if (ss >> x >> comma >> y >> comma >> z >> comma >> yaw) {
                        
                        // 1. Yaw 제어 호출 (0이 아닐 경우)
                        call_yaw_command(yaw);

                        // 2. XYZ 위치 제어 (0이 아닐 경우)
                        if (std::abs(x) > 0.01 || std::abs(y) > 0.01 || std::abs(z) > 0.01) {
                            RCLCPP_INFO(this->get_logger(), "New relative position target: x=%.2f, y=%.2f, z=%.2f", x, y, z);
                            target_pose_msg.position.x = origin_pose.pose.position.x + x;
                            target_pose_msg.position.y = origin_pose.pose.position.y + y;
                            target_pose_msg.position.z = origin_pose.pose.position.z + z;

                            currentState = State::MOVING;
                        } else {
                            RCLCPP_INFO(this->get_logger(), "XYZ target unchanged, holding current position.");
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Invalid format. Use: x,y,z,yaw (e.g., 1,2,-0.5,45)");
                    }
                } else if (currentState == State::AWAITING_ORIGIN) {
                    RCLCPP_INFO(this->get_logger(), "Please wait, still setting origin...");
                }

                input_buffer.clear();
            } else {
                input_buffer += buf[0];
            }
        }

        switch (currentState) {
            case State::AWAITING_ORIGIN:
                // 첫 위치 메시지 수신 대기
                if (pose_received && !origin_set) {
                    origin_pose = current_pose_msg;
                    origin_yaw = getYawFromPose(origin_pose);
                    origin_set = true;

                    // 최초 목표 지점 = 원점
                    target_pose_msg.position = origin_pose.pose.position;
                    // target_pose_msg.yaw 는 type_mask로 인해 무시됨
                    RCLCPP_INFO(this->get_logger(), "********************************************");
                    RCLCPP_INFO(this->get_logger(), "Origin Set at [x: %.2f, y: %.2f, z: %.2f, yaw: %.2f rad]", 
                             origin_pose.pose.position.x,
                             origin_pose.pose.position.y,
                             origin_pose.pose.position.z,
                             origin_yaw);
                    RCLCPP_INFO(this->get_logger(), "********************************************");
                    currentState = State::INITIALIZING; // 초기화 상태로 이행
                    init_counter = 0;
                }
                break;
            
            case State::INITIALIZING:
                setpoint_pub->publish(target_pose_msg);

                if (init_counter < INIT_PUBLISH_COUNT) {
                    init_counter++;
                } else {
                    if (current_state_msg.mode != "GUIDED") {
                        if (call_set_mode("GUIDED")) {
                            RCLCPP_INFO(this->get_logger(), "GUIDED mode requested...");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Failed to request GUIDED mode. Retrying...");
                        }
                    } else {
                        RCLCPP_INFO(this->get_logger(), "GUIDED mode active. Holding at Origin.");
                        RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
                        RCLCPP_INFO(this->get_logger(), "Enter new relative command (x,y,z,yaw):");
                        currentState = State::HOLDING;
                    }
                }
                break;

            case State::HOLDING:
                setpoint_pub->publish(target_pose_msg);
                break;
            
            case State::MOVING:
                setpoint_pub->publish(target_pose_msg);

                if (pose_received) {
                    double dx = target_pose_msg.position.x - current_pose_msg.pose.position.x;
                    double dy = target_pose_msg.position.y - current_pose_msg.pose.position.y;
                    double dz = target_pose_msg.position.z - current_pose_msg.pose.position.z;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (dist < ARRIVAL_THRESHOLD) {
                        RCLCPP_INFO(this->get_logger(), "*************************");
                        RCLCPP_INFO(this->get_logger(), "Arrived at XYZ target (%.2f, %.2f, %.2f)", 
                                 target_pose_msg.position.x, 
                                 target_pose_msg.position.y, 
                                 target_pose_msg.position.z);
                        RCLCPP_INFO(this->get_logger(), "Enter new relative command (x,y,z,yaw):");
                        RCLCPP_INFO(this->get_logger(), "*************************");
                        currentState = State::HOLDING;
                    }
                }
                break;
        }

        if (currentState != State::AWAITING_ORIGIN) {
            target_pose_msg.header.stamp = this->now();
        }
    }
    

private:

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client;
    mavros_msgs::msg::PositionTarget target_pose_msg;
    geometry_msgs::msg::PoseStamped current_pose_msg;
    geometry_msgs::msg::PoseStamped origin_pose;
    State currentState = State::AWAITING_ORIGIN;
    mavros_msgs::msg::State current_state_msg;
    std::string input_buffer;

    double origin_yaw = 0.0;
    bool origin_set = false;
    bool pose_received = false;
    const double ARRIVAL_THRESHOLD = 0.1;
    int init_counter = 0;
    const int INIT_PUBLISH_COUNT = 30;


    double getYawFromPose(const geometry_msgs::msg::PoseStamped& pose_msg) {
        tf2::Quaternion q(
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw_val;
        m.getRPY(roll, pitch, yaw_val);
        return yaw_val;
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_msg = *msg;
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_msg = *msg;
        
        if (!pose_received) {
            pose_received = true;
        }
    }

    void setup_nonblocking_stdin() {
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void call_yaw_command(double relative_yaw_deg) {
        using namespace std::chrono_literals;
        if (std::abs(relative_yaw_deg) < 0.01) {
            return;
        }

        double rotation_direction = 0.0;
        if (relative_yaw_deg > 0) {
            rotation_direction = 1.0;
        } else {
            rotation_direction = -1.0;
        }

        double target_angle_deg = std::abs(relative_yaw_deg);

        RCLCPP_INFO(this->get_logger(), "Requesting relative YAW change: %.2f deg (Direction: %s)",
             relative_yaw_deg, (rotation_direction > 0) ? "CW" : "CCW");

        if (!command_client->wait_for_service(0s)) {
            RCLCPP_WARN(this->get_logger(), "MAV_CMD_CONDITION_YAW service not available.");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->broadcast = false;
        request->command = 115;
        request->confirmation = 0;
        request->param1 = target_angle_deg;
        request->param2 = 10.0;
        request->param3 = rotation_direction;
        request->param4 = 1.0;
        request->param5 = 0.0;
        request->param6 = 0.0;
        request->param7 = 0.0;

        auto future = command_client->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(shared_from_this(), future, 1s);
        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(this->get_logger(), "Yaw command failed with result: %d", (int)response->result);
            } else {
                RCLCPP_INFO(this->get_logger(), "Yaw command sent successfully.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Yaw command request timed out.");
        }
    }

    bool call_set_mode(const std::string& mode) {
        using namespace std::chrono_literals;
        if (!set_mode_client->wait_for_service(0s)) {
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        auto future = set_mode_client->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(shared_from_this(), future, 1s);
        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            return false;
        }

        return future.get()->mode_sent;
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuidedControlNode>();
    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->loop();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
