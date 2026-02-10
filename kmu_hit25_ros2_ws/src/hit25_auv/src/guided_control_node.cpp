#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class GuidedControlNode : public rclcpp::Node
{
public:
  GuidedControlNode()
  : Node("guided_xyz_yaw_node")
  {
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, std::bind(&GuidedControlNode::stateCb, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&GuidedControlNode::poseCb, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    setup_nonblocking_stdin();

    target_pose_msg_.header.frame_id = "map";
    target_pose_msg_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    target_pose_msg_.type_mask = 2496; // position only
    target_pose_msg_.velocity.x = 0.0;
    target_pose_msg_.velocity.y = 0.0;
    target_pose_msg_.velocity.z = 0.0;
    target_pose_msg_.acceleration_or_force.x = 0.0;
    target_pose_msg_.acceleration_or_force.y = 0.0;
    target_pose_msg_.acceleration_or_force.z = 0.0;
    target_pose_msg_.yaw = 0.0;
    target_pose_msg_.yaw_rate = 0.0;

    timer_ = create_wall_timer(100ms, std::bind(&GuidedControlNode::loop, this));

    RCLCPP_INFO(get_logger(), "===== GUIDED Mode XYZ + YAW Controller =====");
    RCLCPP_INFO(get_logger(), "Waiting for first position message to set origin...");
  }

private:
  enum class State { AWAITING_ORIGIN, INITIALIZING, HOLDING, MOVING };

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  State current_state_{State::AWAITING_ORIGIN};
  mavros_msgs::msg::State current_state_msg_;
  geometry_msgs::msg::PoseStamped current_pose_msg_;
  mavros_msgs::msg::PositionTarget target_pose_msg_;

  geometry_msgs::msg::PoseStamped origin_pose_;
  double origin_yaw_{0.0};
  bool origin_set_{false};
  bool pose_received_{false};

  const double ARRIVAL_THRESHOLD = 0.1;
  int init_counter_{0};
  const int INIT_PUBLISH_COUNT = 30;

  std::string input_buffer_;

  static void setup_nonblocking_stdin()
  {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  static double getYawFromPose(const geometry_msgs::msg::Pose &pose)
  {
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void stateCb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_msg_ = *msg;
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_msg_ = *msg;
    pose_received_ = true;
  }

  void request_guided_mode()
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "SetMode service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "GUIDED";
    set_mode_client_->async_send_request(
      req, [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      auto resp = future.get();
      if (resp->mode_sent) {
        RCLCPP_INFO(get_logger(), "GUIDED mode requested...");
      } else {
        RCLCPP_WARN(get_logger(), "Failed to request GUIDED mode. Retrying...");
      }
    });
  }

  void call_yaw_command(double relative_yaw_deg)
  {
    if (std::abs(relative_yaw_deg) < 0.01) {
      return;
    }
    if (!command_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "CommandLong service not ready");
      return;
    }

    double rotation_direction = (relative_yaw_deg > 0) ? 1.0 : -1.0;
    double target_angle_deg = std::abs(relative_yaw_deg);

    RCLCPP_INFO(get_logger(), "Requesting relative YAW change: %.2f deg (Direction: %s)",
                relative_yaw_deg, (rotation_direction > 0) ? "CW" : "CCW");

    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->broadcast = false;
    req->command = 115; // MAV_CMD_CONDITION_YAW
    req->confirmation = 0;
    req->param1 = target_angle_deg;
    req->param2 = 10.0;
    req->param3 = rotation_direction;
    req->param4 = 1.0;
    req->param5 = 0.0;
    req->param6 = 0.0;
    req->param7 = 0.0;

    command_client_->async_send_request(
      req, [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
      auto resp = future.get();
      if (!resp->success) {
        RCLCPP_WARN(get_logger(), "Yaw command failed with result: %d", (int)resp->result);
      } else {
        RCLCPP_INFO(get_logger(), "Yaw command sent successfully.");
      }
    });
  }

  void handle_input()
  {
    char buf[1];
    int n = read(STDIN_FILENO, buf, 1);
    if (n <= 0) {
      return;
    }

    if (buf[0] == '\n') {
      if (current_state_ == State::HOLDING || current_state_ == State::MOVING) {
        double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
        char comma;
        std::stringstream ss(input_buffer_);
        if (ss >> x >> comma >> y >> comma >> z >> comma >> yaw) {
          call_yaw_command(yaw);

          if (std::abs(x) > 0.01 || std::abs(y) > 0.01 || std::abs(z) > 0.01) {
            RCLCPP_INFO(get_logger(), "New relative position target: x=%.2f, y=%.2f, z=%.2f", x, y, z);
            target_pose_msg_.position.x = origin_pose_.pose.position.x + x;
            target_pose_msg_.position.y = origin_pose_.pose.position.y + y;
            target_pose_msg_.position.z = origin_pose_.pose.position.z + z;
            current_state_ = State::MOVING;
          } else {
            RCLCPP_INFO(get_logger(), "XYZ target unchanged, holding current position.");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Invalid format. Use: x,y,z,yaw (e.g., 1,2,-0.5,45)");
        }
      } else if (current_state_ == State::AWAITING_ORIGIN) {
        RCLCPP_INFO(get_logger(), "Please wait, still setting origin...");
      }
      input_buffer_.clear();
    } else {
      input_buffer_ += buf[0];
    }
  }

  void loop()
  {
    handle_input();

    switch (current_state_) {
      case State::AWAITING_ORIGIN:
        if (pose_received_ && !origin_set_) {
          origin_pose_ = current_pose_msg_;
          origin_yaw_ = getYawFromPose(origin_pose_.pose);
          origin_set_ = true;

          target_pose_msg_.position = origin_pose_.pose.position;
          target_pose_msg_.yaw = origin_yaw_;

          RCLCPP_INFO(get_logger(), "********************************************");
          RCLCPP_INFO(get_logger(), "Origin Set at [x: %.2f, y: %.2f, z: %.2f, yaw: %.2f rad]",
                      origin_pose_.pose.position.x,
                      origin_pose_.pose.position.y,
                      origin_pose_.pose.position.z,
                      origin_yaw_);
          RCLCPP_INFO(get_logger(), "********************************************");

          current_state_ = State::INITIALIZING;
          init_counter_ = 0;
        }
        break;

      case State::INITIALIZING:
        setpoint_pub_->publish(target_pose_msg_);
        if (init_counter_ < INIT_PUBLISH_COUNT) {
          init_counter_++;
        } else {
          if (current_state_msg_.mode != "GUIDED") {
            request_guided_mode();
          } else {
            RCLCPP_INFO(get_logger(), "GUIDED mode active. Holding Origin Position and Yaw.");
            RCLCPP_INFO(get_logger(), "--------------------------------------------");
            RCLCPP_INFO(get_logger(), "Enter new relative position and yaw (x,y,z,yaw):");
            current_state_ = State::HOLDING;
          }
        }
        break;

      case State::HOLDING:
        setpoint_pub_->publish(target_pose_msg_);
        break;

      case State::MOVING:
        setpoint_pub_->publish(target_pose_msg_);
        if (pose_received_) {
          double dx = target_pose_msg_.position.x - current_pose_msg_.pose.position.x;
          double dy = target_pose_msg_.position.y - current_pose_msg_.pose.position.y;
          double dz = target_pose_msg_.position.z - current_pose_msg_.pose.position.z;
          double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
          if (dist < ARRIVAL_THRESHOLD) {
            RCLCPP_INFO(get_logger(), "*************************");
            RCLCPP_INFO(get_logger(), "Arrived at target (%.2f, %.2f, %.2f)",
                        target_pose_msg_.position.x,
                        target_pose_msg_.position.y,
                        target_pose_msg_.position.z);
            RCLCPP_INFO(get_logger(), "Enter new relative position and yaw (x,y,z,yaw):");
            current_state_ = State::HOLDING;
          }
        }
        break;
    }

    if (current_state_ != State::AWAITING_ORIGIN) {
      target_pose_msg_.header.stamp = now();
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GuidedControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
