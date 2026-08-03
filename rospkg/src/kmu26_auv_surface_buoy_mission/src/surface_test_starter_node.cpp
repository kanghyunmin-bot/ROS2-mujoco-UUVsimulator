#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace kmu26_auv_surface_buoy_mission
{

/**
 * @brief 수중 레인 제어기 없이 수면 미션만 시험할 때 필요한 계약을 발행한다.
 *
 * 시뮬레이터의 odometry/depth와 두 YOLO 노드의 모델 준비 신호를 확인한
 * 뒤에만 /mission/surface_start를 한 번 발행한다. 따라서 detector가 모델을 로드하는
 * 동안 수면 미션이 먼저 움직이는 경합을 피할 수 있다.
 */
class SurfaceTestStarterNode final : public rclcpp::Node
{
public:
  SurfaceTestStarterNode()
  : Node("surface_test_starter_node")
  {
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/sim/odom");
    depth_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    front_ready_topic_ = declare_parameter<std::string>(
      "front_ready_topic", "/vision/surface/front/ready");
    top_ready_topic_ = declare_parameter<std::string>(
      "top_ready_topic", "/vision/surface/top/ready");
    start_frame_topic_ = declare_parameter<std::string>("start_frame_topic", "/start_frame");
    arena_topic_ = declare_parameter<std::string>("arena_config_topic", "/mission/arena_config");
    surface_start_topic_ = declare_parameter<std::string>(
      "surface_start_topic", "/mission/surface_start");
    require_vision_ = declare_parameter<bool>("require_vision", true);
    require_armed_ = declare_parameter<bool>("require_armed", true);
    required_mode_ = declare_parameter<std::string>("required_mode", "STABILIZE");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    ready_delay_sec_ = declare_parameter<double>("ready_delay_sec", 1.0);

    start_x_m_ = declare_parameter<double>("start_x_m", -15.881);
    start_y_m_ = declare_parameter<double>("start_y_m", 1.305);
    start_yaw_rad_ = declare_parameter<double>("start_yaw_rad", 0.0);
    work_depth_m_ = declare_parameter<double>("work_depth_m", 0.85);

    arena_length_m_ = declare_parameter<double>("arena_length_m", 17.5);
    arena_width_m_ = declare_parameter<double>("arena_width_m", 30.0);
    arena_offset_x_m_ = declare_parameter<double>("arena_offset_x_m", -1.619);
    arena_offset_y_m_ = declare_parameter<double>("arena_offset_y_m", 13.695);
    arena_safety_margin_m_ = declare_parameter<double>("arena_safety_margin_m", 0.45);
    lane_search_offset_m_ = declare_parameter<double>("lane_search_offset_m", 3.6375);
    start_corner_ = declare_parameter<int>("start_corner", 0);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    start_frame_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(start_frame_topic_, latched);
    arena_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(arena_topic_, latched);
    surface_start_pub_ = create_publisher<std_msgs::msg::String>(surface_start_topic_, latched);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr) {have_odom_ = true;});
    depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      depth_topic_, 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {have_depth_ = true;});
    front_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      front_ready_topic_, latched,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {front_vision_ready_ = msg->data;});
    top_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      top_ready_topic_, latched,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {top_vision_ready_ = msg->data;});
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        armed_ = msg->armed;
        mode_ = msg->mode;
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&SurfaceTestStarterNode::on_timer, this));
    RCLCPP_INFO(
      get_logger(), "Waiting for simulator%s%s before starting the surface-only test",
      require_vision_ ? ", front/top YOLO" : "",
      require_armed_ ? ", and armed STABILIZE" : "");
  }

private:
  void publish_contract()
  {
    geometry_msgs::msg::PoseStamped start_frame;
    start_frame.header.stamp = now();
    start_frame.header.frame_id = "world";
    start_frame.pose.position.x = start_x_m_;
    start_frame.pose.position.y = start_y_m_;
    start_frame.pose.orientation.z = std::sin(0.5 * start_yaw_rad_);
    start_frame.pose.orientation.w = std::cos(0.5 * start_yaw_rad_);
    start_frame_pub_->publish(start_frame);

    std_msgs::msg::Float64MultiArray arena;
    arena.data = {
      arena_length_m_, arena_width_m_, arena_offset_x_m_, arena_offset_y_m_,
      arena_safety_margin_m_, lane_search_offset_m_, static_cast<double>(start_corner_)};
    arena_pub_->publish(arena);
  }

  void on_timer()
  {
    if (start_sent_) {
      return;
    }
    // transient_local이지만 mission node의 초기화 순서와 무관하게 받을 수 있도록
    // 시작 전까지 좌표 계약을 반복 발행한다.
    publish_contract();

    const bool vision_ready = !require_vision_ || (front_vision_ready_ && top_vision_ready_);
    const bool vehicle_ready =
      !require_armed_ || (armed_ && (required_mode_.empty() || mode_ == required_mode_));
    if (!have_odom_ || !have_depth_ || !vision_ready || !vehicle_ready) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Surface test waiting: odom=%d depth=%d front_yolo=%d top_yolo=%d armed=%d mode=%s",
        have_odom_, have_depth_, front_vision_ready_, top_vision_ready_, armed_, mode_.c_str());
      ready_since_.reset();
      return;
    }

    if (!ready_since_) {
      ready_since_ = std::chrono::steady_clock::now();
      return;
    }
    const auto ready_for = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *ready_since_).count();
    if (ready_for < ready_delay_sec_) {
      return;
    }

    std_msgs::msg::String start;
    start.data = "cycle=1;final=true;work_depth=" + std::to_string(work_depth_m_);
    surface_start_pub_->publish(start);
    start_sent_ = true;
    RCLCPP_INFO(get_logger(), "Surface-only mission start contract published: %s", start.data.c_str());
  }

  std::string odometry_topic_, depth_topic_, front_ready_topic_, top_ready_topic_;
  std::string start_frame_topic_, arena_topic_, surface_start_topic_;
  std::string state_topic_, required_mode_, mode_;
  bool require_vision_{true}, require_armed_{true}, armed_{false};
  double ready_delay_sec_{1.0};
  double start_x_m_{-15.881}, start_y_m_{1.305}, start_yaw_rad_{0.0}, work_depth_m_{0.85};
  double arena_length_m_{17.5}, arena_width_m_{30.0};
  double arena_offset_x_m_{-1.619}, arena_offset_y_m_{13.695};
  double arena_safety_margin_m_{0.45}, lane_search_offset_m_{3.6375};
  int start_corner_{0};

  bool have_odom_{false}, have_depth_{false};
  bool front_vision_ready_{false}, top_vision_ready_{false}, start_sent_{false};
  std::optional<std::chrono::steady_clock::time_point> ready_since_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_frame_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arena_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr surface_start_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr front_ready_sub_, top_ready_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kmu26_auv_surface_buoy_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kmu26_auv_surface_buoy_mission::SurfaceTestStarterNode>());
  rclcpp::shutdown();
  return 0;
}
