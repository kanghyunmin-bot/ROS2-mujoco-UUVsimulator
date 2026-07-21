#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class BuoyPositionControlNode : public rclcpp::Node {
public:
  BuoyPositionControlNode()
  : Node("buoy_position_control"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    buoy_topic_ = declare_parameter<std::string>("buoy_topic", "/buoy");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    setpoint_topic_ = declare_parameter<std::string>(
      "setpoint_topic", "/mavros/setpoint_raw/local");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    set_mode_service_ = declare_parameter<std::string>(
      "set_mode_service", "/mavros/set_mode");
    hold_mode_ = declare_parameter<std::string>("hold_mode", "ALT_HOLD");
    guided_mode_ = declare_parameter<std::string>("guided_mode", "GUIDED");
    arrival_radius_ = declare_parameter<double>("arrival_radius", 0.10);
    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate", 10.0);
    use_buoy_z_ = declare_parameter<bool>("use_buoy_z", false);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.1);
    mode_request_interval_sec_ = declare_parameter<double>("mode_request_interval_sec", 2.0);

    if (arrival_radius_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "arrival_radius must be positive. Falling back to 0.10 m.");
      arrival_radius_ = 0.10;
    }
    if (setpoint_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "setpoint_rate must be positive. Falling back to 10 Hz.");
      setpoint_rate_hz_ = 10.0;
    }
    if (transform_timeout_sec_ < 0.0) {
      transform_timeout_sec_ = 0.0;
    }
    if (mode_request_interval_sec_ < 0.1) {
      mode_request_interval_sec_ = 0.1;
    }

    const auto qos = rclcpp::QoS(10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      std::bind(&BuoyPositionControlNode::odomCallback, this, std::placeholders::_1));
    buoy_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      buoy_topic_, qos,
      std::bind(&BuoyPositionControlNode::buoyCallback, this, std::placeholders::_1));
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, qos,
      std::bind(&BuoyPositionControlNode::stateCallback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(setpoint_topic_, qos);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service_);

    const auto period = std::chrono::duration<double>(1.0 / setpoint_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BuoyPositionControlNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "Buoy position control ready. odom=%s buoy=%s hold_mode=%s guided_mode=%s "
      "arrival_radius=%.2f use_buoy_z=%s",
      odom_topic_.c_str(), buoy_topic_.c_str(), hold_mode_.c_str(), guided_mode_.c_str(),
      arrival_radius_, use_buoy_z_ ? "true" : "false");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ = msg->pose.pose.position;
    have_odom_ = true;

    if (!have_hold_position_) {
      hold_position_ = current_position_;
      have_hold_position_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Initial hold position set from %s: x=%.2f y=%.2f z=%.2f",
        odom_topic_.c_str(), hold_position_.x, hold_position_.y, hold_position_.z);
    }
  }

  void buoyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received buoy target, but no odometry has arrived yet.");
      return;
    }

    geometry_msgs::msg::PointStamped target_in_odom;
    if (!transformBuoyTarget(*msg, target_in_odom)) {
      return;
    }

    target_position_ = target_in_odom.point;
    if (!use_buoy_z_) {
      target_position_.z = current_position_.z;
    }

    target_active_ = true;
    RCLCPP_INFO(
      get_logger(),
      "New buoy target: x=%.2f y=%.2f z=%.2f (%s radius %.2f m)",
      target_position_.x, target_position_.y, target_position_.z,
      use_buoy_z_ ? "3D" : "XY", arrival_radius_);
  }

  bool transformBuoyTarget(
    const geometry_msgs::msg::PointStamped & input,
    geometry_msgs::msg::PointStamped & output)
  {
    if (input.header.frame_id.empty() || input.header.frame_id == odom_frame_) {
      output = input;
      output.header.frame_id = odom_frame_;
      return true;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        odom_frame_,
        input.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      tf2::doTransform(input, output, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform buoy target from %s to %s: %s",
        input.header.frame_id.c_str(), odom_frame_.c_str(), ex.what());
      return false;
    }
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_mode_ = msg->mode;
    have_state_ = true;
  }

  void controlLoop()
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for odometry on %s before publishing setpoints.",
        odom_topic_.c_str());
      return;
    }

    if (target_active_) {
      requestMode(guided_mode_);
      publishSetpoint(target_position_);

      const auto distance = distanceToTarget(target_position_);
      if (distance <= arrival_radius_) {
        target_active_ = false;
        hold_position_ = current_position_;
        have_hold_position_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Arrived at buoy target within %.2f m (distance %.3f m). Switching to %s hold.",
          arrival_radius_, distance, hold_mode_.c_str());
        requestMode(hold_mode_, true);
      }
      return;
    }

    if (!have_hold_position_) {
      hold_position_ = current_position_;
      have_hold_position_ = true;
    }

    requestMode(hold_mode_);
    publishSetpoint(hold_position_);
  }

  void publishSetpoint(const geometry_msgs::msg::Point & position)
  {
    mavros_msgs::msg::PositionTarget target;
    target.header.stamp = now();
    target.header.frame_id = odom_frame_;
    target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_VX |
      mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    target.position = position;
    setpoint_pub_->publish(target);
  }

  double distanceToTarget(const geometry_msgs::msg::Point & target) const
  {
    const auto dx = target.x - current_position_.x;
    const auto dy = target.y - current_position_.y;
    if (!use_buoy_z_) {
      return std::sqrt(dx * dx + dy * dy);
    }
    const auto dz = target.z - current_position_.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void requestMode(const std::string & mode, bool force = false)
  {
    if (mode.empty()) {
      return;
    }
    if (have_state_ && current_mode_ == mode) {
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    if (!force && last_mode_request_time_.time_since_epoch().count() != 0) {
      const auto elapsed =
        std::chrono::duration<double>(steady_now - last_mode_request_time_).count();
      if (elapsed < mode_request_interval_sec_) {
        return;
      }
    }

    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "SetMode service %s is not ready.", set_mode_service_.c_str());
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    last_mode_request_time_ = steady_now;

    set_mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        const auto response = future.get();
        if (!response->mode_sent) {
          RCLCPP_WARN(get_logger(), "Mode request %s was rejected.", mode.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "Mode request sent: %s", mode.c_str());
        }
      });
  }

  std::string odom_topic_;
  std::string buoy_topic_;
  std::string odom_frame_;
  std::string setpoint_topic_;
  std::string state_topic_;
  std::string set_mode_service_;
  std::string hold_mode_;
  std::string guided_mode_;
  double arrival_radius_{0.10};
  double setpoint_rate_hz_{10.0};
  bool use_buoy_z_{false};
  double transform_timeout_sec_{0.1};
  double mode_request_interval_sec_{2.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr buoy_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Point hold_position_;
  geometry_msgs::msg::Point target_position_;
  bool have_odom_{false};
  bool have_hold_position_{false};
  bool target_active_{false};
  bool have_state_{false};
  std::string current_mode_;
  std::chrono::steady_clock::time_point last_mode_request_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BuoyPositionControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
