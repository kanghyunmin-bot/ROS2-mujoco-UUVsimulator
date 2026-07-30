#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace
{

bool is_finite(double value)
{
  return std::isfinite(value);
}

double linear_speed(const nav_msgs::msg::Odometry & odom)
{
  const auto & velocity = odom.twist.twist.linear;
  return std::sqrt(
    velocity.x * velocity.x +
    velocity.y * velocity.y +
    velocity.z * velocity.z);
}

double position_norm(const nav_msgs::msg::Odometry & odom)
{
  const auto & position = odom.pose.pose.position;
  return std::sqrt(
    position.x * position.x +
    position.y * position.y +
    position.z * position.z);
}

double position_distance(
  const nav_msgs::msg::Odometry & lhs,
  const nav_msgs::msg::Odometry & rhs)
{
  const auto & a = lhs.pose.pose.position;
  const auto & b = rhs.pose.pose.position;
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

using RotationMatrix = std::array<std::array<double, 3>, 3>;

RotationMatrix quaternion_rotation(
  const geometry_msgs::msg::Quaternion & orientation)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;

  return {{
    {{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)}},
    {{2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)}},
    {{2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}},
  }};
}

geometry_msgs::msg::Vector3 rotate_vector(
  const RotationMatrix & rotation,
  const geometry_msgs::msg::Vector3 & vector)
{
  geometry_msgs::msg::Vector3 output;
  output.x =
    rotation[0][0] * vector.x + rotation[0][1] * vector.y +
    rotation[0][2] * vector.z;
  output.y =
    rotation[1][0] * vector.x + rotation[1][1] * vector.y +
    rotation[1][2] * vector.z;
  output.z =
    rotation[2][0] * vector.x + rotation[2][1] * vector.y +
    rotation[2][2] * vector.z;
  return output;
}

void rotate_linear_covariance(
  const RotationMatrix & rotation,
  const std::array<double, 36> & source,
  std::array<double, 36> & destination)
{
  double intermediate[3][3] {};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t index = 0; index < 3; ++index) {
        intermediate[row][column] +=
          rotation[row][index] * source[index * 6 + column];
      }
    }
  }

  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      destination[row * 6 + column] = 0.0;
      for (std::size_t index = 0; index < 3; ++index) {
        destination[row * 6 + column] +=
          intermediate[row][index] * rotation[column][index];
      }
    }
  }
}

std::string default_counter_file()
{
  if (const char * ros_home = std::getenv("ROS_HOME")) {
    if (ros_home[0] != '\0') {
      return (std::filesystem::path(ros_home) / "auv_external_nav_reset_counter").string();
    }
  }
  if (const char * user_home = std::getenv("HOME")) {
    if (user_home[0] != '\0') {
      return (std::filesystem::path(user_home) / ".ros" /
             "auv_external_nav_reset_counter").string();
    }
  }
  return "/tmp/auv_external_nav_reset_counter";
}

}  // namespace

class EkfOdometryGateway : public rclcpp::Node
{
public:
  EkfOdometryGateway()
  : Node("external_nav_odometry_gateway")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odometry/filtered");
    pose_output_topic_ = declare_parameter<std::string>(
      "pose_output_topic", "/mavros/vision_pose/pose_cov");
    velocity_output_topic_ = declare_parameter<std::string>(
      "velocity_output_topic", "/mavros/vision_speed/speed_twist_cov");
    dvl_topic_ = declare_parameter<std::string>("dvl_topic", "/dvl/twist");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    reset_counter_topic_ = declare_parameter<std::string>(
      "reset_counter_topic", "/mavros/vision_pose/reset_counter");
    ready_topic_ = declare_parameter<std::string>(
      "ready_topic", "/external_nav/ready");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/external_nav/status");
    expected_frame_id_ = declare_parameter<std::string>("expected_frame_id", "odom");
    expected_child_frame_id_ = declare_parameter<std::string>(
      "expected_child_frame_id", "base_link");
    counter_file_ = declare_parameter<std::string>(
      "reset_counter_file", default_counter_file());

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    max_odom_age_s_ = declare_parameter<double>("max_odom_age_s", 0.20);
    max_dvl_age_s_ = declare_parameter<double>("max_dvl_age_s", 0.30);
    max_fcu_state_age_s_ = declare_parameter<double>("max_fcu_state_age_s", 3.0);
    stale_pose_covariance_ = declare_parameter<double>(
      "stale_pose_covariance", 1000.0);
    stale_twist_covariance_ = declare_parameter<double>(
      "stale_twist_covariance", 1000.0);
    startup_origin_tolerance_m_ = declare_parameter<double>(
      "startup_origin_tolerance_m", 0.25);
    startup_speed_tolerance_mps_ = declare_parameter<double>(
      "startup_speed_tolerance_mps", 0.15);
    max_position_jump_m_ = declare_parameter<double>("max_position_jump_m", 0.50);
    reset_settle_time_s_ = declare_parameter<double>("reset_settle_time_s", 0.25);
    require_disarmed_for_reset_ = declare_parameter<bool>(
      "require_disarmed_for_reset", true);

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto reliable_qos = rclcpp::QoS(10).reliable();
    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_output_topic_, reliable_qos);
    velocity_pub_ =
      create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      velocity_output_topic_, reliable_qos);
    reset_counter_pub_ = create_publisher<std_msgs::msg::UInt8>(
      reset_counter_topic_, latched_qos);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, latched_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, latched_qos);

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, sensor_qos,
      std::bind(&EkfOdometryGateway::odometry_callback, this, std::placeholders::_1));
    dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      dvl_topic_, sensor_qos,
      std::bind(&EkfOdometryGateway::dvl_callback, this, std::placeholders::_1));
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, reliable_qos,
      std::bind(&EkfOdometryGateway::state_callback, this, std::placeholders::_1));

    const double safe_rate_hz = std::clamp(publish_rate_hz_, 4.0, 100.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / safe_rate_hz),
      std::bind(&EkfOdometryGateway::timer_callback, this));

    last_odom_rx_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_dvl_rx_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_fcu_state_rx_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    reset_sent_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    set_status(false, "WAITING: FCU, DVL and EKF must become ready while disarmed");
    RCLCPP_INFO(
      get_logger(),
      "ExternalNav gateway: %s -> %s + %s, DVL watchdog=%s",
      input_topic_.c_str(), pose_output_topic_.c_str(),
      velocity_output_topic_.c_str(), dvl_topic_.c_str());
  }

private:
  enum class GateState
  {
    WAITING,
    RESET_SETTLING,
    ACTIVE
  };

  bool valid_odometry(const nav_msgs::msg::Odometry & odom) const
  {
    if (
      odom.header.frame_id != expected_frame_id_ ||
      odom.child_frame_id != expected_child_frame_id_)
    {
      return false;
    }

    const auto & position = odom.pose.pose.position;
    const auto & orientation = odom.pose.pose.orientation;
    const auto & linear = odom.twist.twist.linear;
    const auto & angular = odom.twist.twist.angular;
    if (
      !is_finite(position.x) || !is_finite(position.y) || !is_finite(position.z) ||
      !is_finite(orientation.x) || !is_finite(orientation.y) ||
      !is_finite(orientation.z) || !is_finite(orientation.w) ||
      !is_finite(linear.x) || !is_finite(linear.y) || !is_finite(linear.z) ||
      !is_finite(angular.x) || !is_finite(angular.y) || !is_finite(angular.z))
    {
      return false;
    }

    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (quaternion_norm < 0.5 || odom.header.stamp.sec == 0) {
      return false;
    }

    return is_finite(odom.pose.covariance[0]) &&
           is_finite(odom.pose.covariance[7]) &&
           is_finite(odom.pose.covariance[14]) &&
           is_finite(odom.twist.covariance[0]) &&
           is_finite(odom.twist.covariance[7]) &&
           is_finite(odom.twist.covariance[14]);
  }

  bool valid_dvl_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist) const
  {
    const auto & linear = twist.twist.twist.linear;
    return is_finite(linear.x) && is_finite(linear.y) && is_finite(linear.z) &&
           is_finite(twist.twist.covariance[0]) &&
           is_finite(twist.twist.covariance[7]) &&
           is_finite(twist.twist.covariance[14]);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!valid_odometry(*msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid EKF odometry; the last valid estimate remains active");
      return;
    }

    if (
      gate_state_ == GateState::ACTIVE && have_previous_odom_ &&
      max_position_jump_m_ > 0.0 &&
      position_distance(*msg, previous_odom_) > max_position_jump_m_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "EKF position jump exceeded %.2f m; accepting it for autonomous recovery",
        max_position_jump_m_);
    }

    latest_odom_ = *msg;
    previous_odom_ = *msg;
    have_previous_odom_ = true;
    have_valid_odom_ = true;
    last_odom_rx_ = now();
  }

  void dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    if (!valid_dvl_twist(*msg)) {
      return;
    }
    last_dvl_rx_ = now();
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool lost_connection =
      have_fcu_state_ && fcu_connected_ && !msg->connected;

    have_fcu_state_ = true;
    fcu_connected_ = msg->connected;
    fcu_armed_ = msg->armed;
    last_fcu_state_rx_ = now();

    if (
      lost_connection &&
      (gate_state_ == GateState::RESET_SETTLING ||
      gate_state_ == GateState::ACTIVE))
    {
      RCLCPP_WARN(
        get_logger(),
        "FCU disconnected after ExternalNav reset; odometry publication continues");
    }
  }

  double age_seconds(const rclcpp::Time & stamp, const rclcpp::Time & current) const
  {
    if (stamp.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return (current - stamp).seconds();
  }

  bool data_is_fresh(const rclcpp::Time & current)
  {
    return have_valid_odom_ &&
           age_seconds(last_odom_rx_, current) <= max_odom_age_s_ &&
           age_seconds(last_dvl_rx_, current) <= max_dvl_age_s_;
  }

  bool startup_conditions_met(const rclcpp::Time & current)
  {
    if (
      !have_fcu_state_ || !fcu_connected_ ||
      age_seconds(last_fcu_state_rx_, current) > max_fcu_state_age_s_ ||
      !data_is_fresh(current))
    {
      return false;
    }
    if (require_disarmed_for_reset_ && fcu_armed_) {
      return false;
    }
    return position_norm(latest_odom_) <= startup_origin_tolerance_m_ &&
           linear_speed(latest_odom_) <= startup_speed_tolerance_mps_;
  }

  uint8_t next_reset_counter()
  {
    int last_counter = 0;
    {
      std::ifstream input(counter_file_);
      if (!(input >> last_counter) || last_counter < 0 || last_counter > 255) {
        last_counter = 0;
      }
    }

    const uint8_t next = static_cast<uint8_t>((last_counter + 1) & 0xFF);
    const std::filesystem::path counter_path(counter_file_);
    std::error_code error;
    if (!counter_path.parent_path().empty()) {
      std::filesystem::create_directories(counter_path.parent_path(), error);
      if (error) {
        throw std::runtime_error(
                "failed to create reset counter directory: " + error.message());
      }
    }

    auto temporary_path = counter_path;
    temporary_path += ".tmp";
    {
      std::ofstream output(temporary_path, std::ios::trunc);
      if (!output || !(output << static_cast<int>(next) << '\n')) {
        throw std::runtime_error("failed to write reset counter file");
      }
    }
    std::filesystem::rename(temporary_path, counter_path, error);
    if (error) {
      std::filesystem::remove(counter_path, error);
      error.clear();
      std::filesystem::rename(temporary_path, counter_path, error);
      if (error) {
        throw std::runtime_error(
                "failed to install reset counter file: " + error.message());
      }
    }
    return next;
  }

  void begin_reset(const rclcpp::Time & current)
  {
    try {
      reset_counter_ = next_reset_counter();
    } catch (const std::exception & error) {
      set_status(
        false,
        std::string("WAITING: could not persist reset counter: ") + error.what());
      return;
    }

    std_msgs::msg::UInt8 counter_message;
    counter_message.data = reset_counter_;
    reset_counter_pub_->publish(counter_message);
    reset_sent_time_ = current;
    gate_state_ = GateState::RESET_SETTLING;
    set_status(
      false,
      "RESETTING: published ExternalNav reset_counter=" +
      std::to_string(reset_counter_));
  }

  void publish_external_nav(const rclcpp::Time & current, bool hold_last_estimate)
  {
    const uint64_t source_stamp_ns =
      static_cast<uint64_t>(latest_odom_.header.stamp.sec) * 1000000000ULL +
      latest_odom_.header.stamp.nanosec;
    if (!hold_last_estimate && source_stamp_ns == last_published_source_stamp_ns_) {
      return;
    }

    auto output = latest_odom_;
    if (hold_last_estimate) {
      output.header.stamp = current;
      output.twist.twist.linear.x = 0.0;
      output.twist.twist.linear.y = 0.0;
      output.twist.twist.linear.z = 0.0;
      output.twist.twist.angular.x = 0.0;
      output.twist.twist.angular.y = 0.0;
      output.twist.twist.angular.z = 0.0;

      for (std::size_t index : {0U, 7U, 14U, 21U, 28U, 35U}) {
        output.pose.covariance[index] = std::max(
          output.pose.covariance[index], stale_pose_covariance_);
        output.twist.covariance[index] = std::max(
          output.twist.covariance[index], stale_twist_covariance_);
      }
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = output.header;
    pose.pose = output.pose;

    geometry_msgs::msg::TwistWithCovarianceStamped velocity;
    velocity.header = output.header;
    velocity.twist = output.twist;

    // nav_msgs/Odometry expresses twist in child_frame_id. ArduPilot's
    // VISION_SPEED_ESTIMATE expects velocity in the local earth frame, so
    // rotate robot_localization's base_link velocity into odom ENU first.
    const auto rotation = quaternion_rotation(output.pose.pose.orientation);
    velocity.twist.twist.linear =
      rotate_vector(rotation, output.twist.twist.linear);
    velocity.twist.twist.angular =
      rotate_vector(rotation, output.twist.twist.angular);
    rotate_linear_covariance(
      rotation, output.twist.covariance, velocity.twist.covariance);

    pose_pub_->publish(pose);
    velocity_pub_->publish(velocity);
    last_published_source_stamp_ns_ = source_stamp_ns;
  }

  void timer_callback()
  {
    const auto current = now();

    if (gate_state_ == GateState::WAITING) {
      if (!startup_conditions_met(current)) {
        if (fcu_armed_) {
          set_status(false, "WAITING: disarm is required before ExternalNav reset");
        } else {
          set_status(false, "WAITING: fixed-origin EKF, DVL and FCU are not ready");
        }
        return;
      }
      begin_reset(current);
      return;
    }

    if (gate_state_ == GateState::RESET_SETTLING) {
      if (age_seconds(last_fcu_state_rx_, current) > max_fcu_state_age_s_) {
        set_status(false, "RESETTING: waiting for FCU connection");
        return;
      }
      if (require_disarmed_for_reset_ && fcu_armed_) {
        set_status(false, "RESETTING: waiting for disarm");
        return;
      }
      if (!startup_conditions_met(current)) {
        set_status(false, "RESETTING: waiting for stable origin data");
        return;
      }
      if ((current - reset_sent_time_).seconds() < reset_settle_time_s_) {
        return;
      }
      gate_state_ = GateState::ACTIVE;
      set_status(
        true,
        "ACTIVE: filtered pose and world velocity are being sent to Pixhawk");
    }

    const bool fcu_state_stale =
      !fcu_connected_ ||
      age_seconds(last_fcu_state_rx_, current) > max_fcu_state_age_s_;
    const bool odometry_stale =
      !have_valid_odom_ ||
      age_seconds(last_odom_rx_, current) > max_odom_age_s_;
    const bool dvl_stale =
      age_seconds(last_dvl_rx_, current) > max_dvl_age_s_;

    publish_external_nav(current, odometry_stale);

    if (odometry_stale) {
      set_status(
        true,
        "DEGRADED: EKF is stale; holding the last position with high covariance");
    } else if (fcu_state_stale) {
      set_status(
        true,
        "DEGRADED: FCU link is stale; ExternalNav publication continues");
    } else if (dvl_stale) {
      set_status(
        true,
        "DEGRADED: DVL is stale; filtered ExternalNav publication continues");
    } else {
      set_status(
        true,
        "ACTIVE: filtered pose and world velocity are being sent to Pixhawk");
    }
  }

  void set_status(bool ready, const std::string & status)
  {
    if (ready == last_ready_ && status == last_status_) {
      return;
    }
    last_ready_ = ready;
    last_status_ = status;

    std_msgs::msg::Bool ready_message;
    ready_message.data = ready;
    ready_pub_->publish(ready_message);

    std_msgs::msg::String status_message;
    status_message.data = status;
    status_pub_->publish(status_message);

    if (ready) {
      RCLCPP_INFO(get_logger(), "%s", status.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", status.c_str());
    }
  }

  std::string input_topic_;
  std::string pose_output_topic_;
  std::string velocity_output_topic_;
  std::string dvl_topic_;
  std::string state_topic_;
  std::string reset_counter_topic_;
  std::string ready_topic_;
  std::string status_topic_;
  std::string expected_frame_id_;
  std::string expected_child_frame_id_;
  std::string counter_file_;

  double publish_rate_hz_{30.0};
  double max_odom_age_s_{0.2};
  double max_dvl_age_s_{0.3};
  double max_fcu_state_age_s_{3.0};
  double stale_pose_covariance_{1000.0};
  double stale_twist_covariance_{1000.0};
  double startup_origin_tolerance_m_{0.25};
  double startup_speed_tolerance_mps_{0.15};
  double max_position_jump_m_{0.5};
  double reset_settle_time_s_{0.25};
  bool require_disarmed_for_reset_{true};

  GateState gate_state_{GateState::WAITING};
  bool have_fcu_state_{false};
  bool fcu_connected_{false};
  bool fcu_armed_{false};
  bool have_valid_odom_{false};
  bool have_previous_odom_{false};
  uint8_t reset_counter_{0};
  uint64_t last_published_source_stamp_ns_{0};
  bool last_ready_{false};
  std::string last_status_;

  rclcpp::Time last_odom_rx_;
  rclcpp::Time last_dvl_rx_;
  rclcpp::Time last_fcu_state_rx_;
  rclcpp::Time reset_sent_time_;
  nav_msgs::msg::Odometry latest_odom_;
  nav_msgs::msg::Odometry previous_odom_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reset_counter_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfOdometryGateway>());
  rclcpp::shutdown();
  return 0;
}
