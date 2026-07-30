#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <auv_msg/msg/auv_setpoint.hpp>
#include <auv_msg/msg/mission_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{

bool isFinite(double value)
{
  return std::isfinite(value);
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::optional<double> yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (
    !isFinite(orientation.x) || !isFinite(orientation.y) ||
    !isFinite(orientation.z) || !isFinite(orientation.w))
  {
    return std::nullopt;
  }

  const double norm_squared =
    orientation.x * orientation.x +
    orientation.y * orientation.y +
    orientation.z * orientation.z +
    orientation.w * orientation.w;
  if (norm_squared < 1.0e-12) {
    return std::nullopt;
  }

  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = orientation.x * inverse_norm;
  const double y = orientation.y * inverse_norm;
  const double z = orientation.z * inverse_norm;
  const double w = orientation.w * inverse_norm;
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(0.5 * yaw);
  orientation.w = std::cos(0.5 * yaw);
  return orientation;
}

double distance(
  const geometry_msgs::msg::Point & lhs,
  const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double speed(const nav_msgs::msg::Odometry & odometry)
{
  const auto & linear = odometry.twist.twist.linear;
  return std::sqrt(
    linear.x * linear.x +
    linear.y * linear.y +
    linear.z * linear.z);
}

}  // namespace

class GuidedNavigationNode : public rclcpp::Node
{
public:
  GuidedNavigationNode()
  : Node("guided_navigation")
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/guided/goal");
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/waypoint");
    waypoint_enable_topic_ = declare_parameter<std::string>(
      "waypoint_enable_topic", "/guided/waypoint_enable");
    cancel_topic_ = declare_parameter<std::string>("cancel_topic", "/guided/cancel");
    local_position_topic_ = declare_parameter<std::string>(
      "local_position_topic", "/mavros/local_position/odom");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    external_nav_ready_topic_ = declare_parameter<std::string>(
      "external_nav_ready_topic", "/external_nav/ready");
    recapture_start_frame_topic_ = declare_parameter<std::string>(
      "recapture_start_frame_topic", "/guided/recapture_start_frame");
    setpoint_topic_ = declare_parameter<std::string>(
      "setpoint_topic", "/mavros/setpoint_raw/local");
    set_mode_service_ = declare_parameter<std::string>(
      "set_mode_service", "/mavros/set_mode");
    status_topic_ = declare_parameter<std::string>("status_topic", "/guided/status");
    arrived_topic_ = declare_parameter<std::string>("arrived_topic", "/guided/arrived");
    mission_status_topic_ = declare_parameter<std::string>(
      "mission_status_topic", "/guided/mission_status");
    active_target_topic_ = declare_parameter<std::string>(
      "active_target_topic", "/guided/active_target");
    start_frame_topic_ = declare_parameter<std::string>(
      "start_frame_topic", "/guided/start_frame");
    command_frame_ = declare_parameter<std::string>("command_frame", "odom");
    guided_mode_ = declare_parameter<std::string>("guided_mode", "GUIDED");

    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 10.0);
    position_timeout_s_ = declare_parameter<double>("position_timeout_s", 0.5);
    prime_duration_s_ = declare_parameter<double>("prime_duration_s", 0.5);
    mode_request_interval_s_ = declare_parameter<double>(
      "mode_request_interval_s", 1.0);
    progress_status_interval_s_ = declare_parameter<double>(
      "progress_status_interval_s", 1.0);
    arrival_radius_m_ = declare_parameter<double>("arrival_radius_m", 0.20);
    arrival_speed_mps_ = declare_parameter<double>("arrival_speed_mps", 0.10);
    arrival_settle_time_s_ = declare_parameter<double>(
      "arrival_settle_time_s", 1.0);
    arrival_yaw_tolerance_rad_ = declare_parameter<double>(
      "arrival_yaw_tolerance_rad", 0.1745329252);
    align_heading_before_move_ = declare_parameter<bool>(
      "align_heading_before_move", true);
    heading_tolerance_rad_ = declare_parameter<double>(
      "heading_tolerance_rad", 0.0872664626);
    heading_settle_time_s_ = declare_parameter<double>(
      "heading_settle_time_s", 0.5);
    heading_timeout_s_ = declare_parameter<double>(
      "heading_timeout_s", 5.0);
    heading_update_min_distance_m_ = declare_parameter<double>(
      "heading_update_min_distance_m", 0.20);
    max_goal_distance_m_ = declare_parameter<double>("max_goal_distance_m", 20.0);
    min_goal_z_m_ = declare_parameter<double>("min_goal_z_m", -50.0);
    max_goal_z_m_ = declare_parameter<double>("max_goal_z_m", 1.0);
    waypoint_update_epsilon_m_ = declare_parameter<double>(
      "waypoint_update_epsilon_m", 0.02);
    waypoint_cache_timeout_s_ = declare_parameter<double>(
      "waypoint_cache_timeout_s", 1.0);
    restore_guided_mode_ = declare_parameter<bool>("restore_guided_mode", true);
    start_frame_require_disarmed_ = declare_parameter<bool>(
      "start_frame_require_disarmed", true);

    setpoint_rate_hz_ = std::clamp(setpoint_rate_hz_, 2.0, 50.0);
    position_timeout_s_ = std::max(position_timeout_s_, 0.1);
    prime_duration_s_ = std::max(prime_duration_s_, 0.0);
    mode_request_interval_s_ = std::max(mode_request_interval_s_, 0.2);
    progress_status_interval_s_ = std::max(progress_status_interval_s_, 0.1);
    arrival_radius_m_ = std::max(arrival_radius_m_, 0.01);
    arrival_speed_mps_ = std::max(arrival_speed_mps_, 0.01);
    arrival_settle_time_s_ = std::max(arrival_settle_time_s_, 0.0);
    arrival_yaw_tolerance_rad_ = std::clamp(
      arrival_yaw_tolerance_rad_, 0.01, 3.14159265358979323846);
    heading_tolerance_rad_ = std::clamp(
      heading_tolerance_rad_, 0.01, 3.14159265358979323846);
    heading_settle_time_s_ = std::max(heading_settle_time_s_, 0.0);
    heading_timeout_s_ = std::max(heading_timeout_s_, 0.0);
    heading_update_min_distance_m_ = std::max(
      heading_update_min_distance_m_, 0.01);
    waypoint_update_epsilon_m_ = std::max(waypoint_update_epsilon_m_, 0.001);
    waypoint_cache_timeout_s_ = std::max(waypoint_cache_timeout_s_, 0.0);

    auto reliable_qos = rclcpp::QoS(10).reliable();
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

    goal_sub_ = create_subscription<auv_msg::msg::AuvSetpoint>(
      goal_topic_, reliable_qos,
      std::bind(&GuidedNavigationNode::goalCallback, this, std::placeholders::_1));
    waypoint_sub_ = create_subscription<mavros_msgs::msg::PositionTarget>(
      waypoint_topic_, reliable_qos,
      std::bind(&GuidedNavigationNode::waypointCallback, this, std::placeholders::_1));
    waypoint_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      waypoint_enable_topic_, reliable_qos,
      std::bind(
        &GuidedNavigationNode::waypointEnableCallback,
        this, std::placeholders::_1));
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
      cancel_topic_, reliable_qos,
      std::bind(&GuidedNavigationNode::cancelCallback, this, std::placeholders::_1));
    recapture_start_frame_sub_ = create_subscription<std_msgs::msg::Empty>(
      recapture_start_frame_topic_, reliable_qos,
      std::bind(
        &GuidedNavigationNode::recaptureStartFrameCallback,
        this, std::placeholders::_1));
    local_position_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_position_topic_, sensor_qos,
      std::bind(&GuidedNavigationNode::localPositionCallback, this, std::placeholders::_1));
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, reliable_qos,
      std::bind(&GuidedNavigationNode::stateCallback, this, std::placeholders::_1));
    external_nav_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      external_nav_ready_topic_, latched_qos,
      std::bind(&GuidedNavigationNode::externalNavReadyCallback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      setpoint_topic_, sensor_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, latched_qos);
    arrived_pub_ = create_publisher<std_msgs::msg::Bool>(arrived_topic_, latched_qos);
    mission_status_pub_ = create_publisher<auv_msg::msg::MissionStatus>(
      mission_status_topic_, latched_qos);
    active_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      active_target_topic_, latched_qos);
    start_frame_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      start_frame_topic_, latched_qos);

    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service_);

    const auto period = std::chrono::duration<double>(1.0 / setpoint_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GuidedNavigationNode::controlLoop, this));

    last_position_rx_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    phase_started_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    arrival_started_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    heading_aligned_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_progress_status_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_waypoint_rx_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publishArrived(false);
    publishMissionStatus(auv_msg::msg::MissionStatus::READY);
    setStatus("IDLE: waiting for a Guided goal");

    RCLCPP_INFO(
      get_logger(),
      "Guided navigation ready: goal=%s, waypoint=%s, enable=%s, "
      "FCU position=%s, setpoint=%s, frame=%s",
      goal_topic_.c_str(), waypoint_topic_.c_str(), waypoint_enable_topic_.c_str(),
      local_position_topic_.c_str(), setpoint_topic_.c_str(), command_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Goal modes: 0=absolute odom, 1=odom-axis offset, "
      "2=start-frame absolute; heading automatically faces the target");
  }

private:
  enum class ControlState
  {
    IDLE,
    PRIMING,
    WAITING_GUIDED,
    ALIGNING_HEADING,
    MOVING,
    HOLDING
  };

  enum class GoalSource
  {
    NONE,
    DIRECT,
    WAYPOINT
  };

  void goalCallback(const auv_msg::msg::AuvSetpoint::SharedPtr message)
  {
    if (!validGoal(*message)) {
      return;
    }

    geometry_msgs::msg::Point requested;
    if (message->mode == 0) {
      requested.x = message->x;
      requested.y = message->y;
      requested.z = message->z;
    } else if (message->mode == 1) {
      requested.x = current_odometry_.pose.pose.position.x + message->x;
      requested.y = current_odometry_.pose.pose.position.y + message->y;
      requested.z = current_odometry_.pose.pose.position.z + message->z;
    } else {
      const double cosine = std::cos(start_frame_yaw_);
      const double sine = std::sin(start_frame_yaw_);
      requested.x =
        start_frame_origin_.x + cosine * message->x - sine * message->y;
      requested.y =
        start_frame_origin_.y + sine * message->x + cosine * message->y;
      requested.z = start_frame_origin_.z + message->z;
    }

    const double goal_distance = distance(
      requested, current_odometry_.pose.pose.position);
    if (max_goal_distance_m_ > 0.0 && goal_distance > max_goal_distance_m_) {
      rejectGoal(
        "goal is farther than max_goal_distance_m (" +
        std::to_string(goal_distance) + " m)");
      return;
    }
    if (requested.z < min_goal_z_m_ || requested.z > max_goal_z_m_) {
      rejectGoal(
        "goal z is outside configured limits [" +
        std::to_string(min_goal_z_m_) + ", " +
        std::to_string(max_goal_z_m_) + "]");
      return;
    }

    activateTarget(
      requested, GoalSource::DIRECT, goalModeText(message->mode));
  }

  void waypointCallback(
    const mavros_msgs::msg::PositionTarget::SharedPtr message)
  {
    if (!validWaypointMessage(*message)) {
      return;
    }
    pending_waypoint_ = *message;
    last_waypoint_rx_ = now();
    tryActivatePendingWaypoint();
  }

  void waypointEnableCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    const bool was_enabled = waypoint_enabled_;
    waypoint_enabled_ = message->data;
    if (!waypoint_enabled_) {
      if (goal_source_ == GoalSource::WAYPOINT &&
        control_state_ != ControlState::IDLE)
      {
        stopGoal("IDLE: external /waypoint control disabled");
      } else if (was_enabled && control_state_ == ControlState::IDLE) {
        setStatus("IDLE: external /waypoint control disabled");
      }
      return;
    }

    if (!was_enabled && control_state_ == ControlState::IDLE) {
      setStatus("IDLE: /waypoint enabled; waiting for a fresh target");
    }
    tryActivatePendingWaypoint();
  }

  bool validWaypointMessage(
    const mavros_msgs::msg::PositionTarget & message)
  {
    if (
      message.coordinate_frame !=
      mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring /waypoint: coordinate_frame must be FRAME_LOCAL_NED");
      return false;
    }

    const uint16_t ignored_position =
      mavros_msgs::msg::PositionTarget::IGNORE_PX |
      mavros_msgs::msg::PositionTarget::IGNORE_PY |
      mavros_msgs::msg::PositionTarget::IGNORE_PZ;
    if ((message.type_mask & ignored_position) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring /waypoint: x, y, and z positions must all be enabled");
      return false;
    }
    if (
      !isFinite(message.position.x) || !isFinite(message.position.y) ||
      !isFinite(message.position.z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring /waypoint: position contains a non-finite coordinate");
      return false;
    }

    const std::string waypoint_frame = normalizedFrameId(message.header.frame_id);
    const std::string command_frame = normalizedFrameId(command_frame_);
    if (!waypoint_frame.empty() && waypoint_frame != command_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring /waypoint: frame '%s' does not match command frame '%s'",
        message.header.frame_id.c_str(), command_frame_.c_str());
      return false;
    }
    return true;
  }

  void tryActivatePendingWaypoint()
  {
    if (!waypoint_enabled_ || !pending_waypoint_.has_value()) {
      return;
    }

    const auto current_time = now();
    if (
      waypoint_cache_timeout_s_ > 0.0 &&
      (current_time - last_waypoint_rx_).seconds() > waypoint_cache_timeout_s_)
    {
      pending_waypoint_.reset();
      setStatus("IDLE: /waypoint enabled; waiting for a fresh target");
      return;
    }

    geometry_msgs::msg::Point requested = pending_waypoint_->position;
    if (
      goal_source_ == GoalSource::WAYPOINT &&
      control_state_ != ControlState::IDLE &&
      distance(requested, target_position_) <= waypoint_update_epsilon_m_)
    {
      return;
    }

    const auto blocked = targetBlockReason(requested, false);
    if (blocked.has_value()) {
      setStatus("WAITING_WAYPOINT: " + *blocked);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting to accept /waypoint: %s", blocked->c_str());
      return;
    }

    activateTarget(requested, GoalSource::WAYPOINT, "external /waypoint");
  }

  void activateTarget(
    const geometry_msgs::msg::Point & requested,
    GoalSource source,
    const std::string & source_text)
  {
    const double goal_distance = distance(
      requested, current_odometry_.pose.pose.position);
    const bool updating = control_state_ != ControlState::IDLE;

    if (
      updating && source == goal_source_ &&
      distance(requested, target_position_) <= waypoint_update_epsilon_m_)
    {
      return;
    }

    hold_position_ = current_odometry_.pose.pose.position;
    if (!updating) {
      hold_yaw_ = current_yaw_;
    }
    target_position_ = requested;
    target_yaw_ = current_yaw_;
    target_yaw_enabled_ = true;
    updateTargetHeading(false);
    goal_source_ = source;
    arrival_started_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    heading_aligned_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_progress_status_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    if (!updating) {
      control_state_ = ControlState::PRIMING;
      phase_started_at_ = now();
    } else if (control_state_ == ControlState::HOLDING) {
      hold_yaw_ = current_yaw_;
      if (target_yaw_enabled_ && align_heading_before_move_) {
        control_state_ = ControlState::ALIGNING_HEADING;
        phase_started_at_ = now();
      } else {
        control_state_ = ControlState::MOVING;
      }
    }

    publishArrived(false);
    publishMissionStatus(auv_msg::msg::MissionStatus::RUNNING);
    publishActiveTarget();
    if (updating) {
      setStatus(
        "UPDATED: " + source_text + " target " +
        pointText(target_position_));
    } else {
      setStatus(
        "PRIMING: holding current position before " + source_text +
        " target " + pointText(target_position_));
    }

    RCLCPP_INFO(
      get_logger(),
      "%s %s target: odom=(%.3f, %.3f, %.3f), "
      "automatic heading=%.2f deg, distance=%.3f m",
      updating ? "Updated" : "Accepted", source_text.c_str(),
      target_position_.x, target_position_.y, target_position_.z,
      target_yaw_ * 180.0 / 3.14159265358979323846, goal_distance);
  }

  bool validGoal(const auv_msg::msg::AuvSetpoint & message)
  {
    if (
      !isFinite(message.x) || !isFinite(message.y) ||
      !isFinite(message.z))
    {
      rejectGoal("goal contains a non-finite coordinate");
      return false;
    }
    if (message.mode < 0 || message.mode > 2) {
      rejectGoal(
        "mode must be 0 (absolute odom), 1 (odom-axis offset), "
        "or 2 (start-frame absolute)");
      return false;
    }
    if (!have_state_ || !fcu_connected_) {
      rejectGoal("FCU is not connected");
      return false;
    }
    if (!fcu_armed_) {
      rejectGoal("FCU must be armed before accepting a movement goal");
      return false;
    }
    if (!external_nav_ready_) {
      rejectGoal("ExternalNav gateway is not ready");
      return false;
    }
    if (!positionFresh(now())) {
      rejectGoal("Pixhawk local position is not fresh");
      return false;
    }
    if (message.mode == 2 && !start_frame_ready_) {
      rejectGoal(
        "start frame is not ready; wait for ExternalNav while disarmed "
        "or publish /guided/recapture_start_frame");
      return false;
    }
    if (!have_current_yaw_) {
      rejectGoal("Pixhawk local orientation is not valid");
      return false;
    }
    return true;
  }

  std::optional<std::string> targetBlockReason(
    const geometry_msgs::msg::Point & requested,
    bool require_start_frame) const
  {
    if (!have_state_ || !fcu_connected_) {
      return "FCU is not connected";
    }
    if (!fcu_armed_) {
      return "FCU must be armed before accepting a movement goal";
    }
    if (!external_nav_ready_) {
      return "ExternalNav gateway is not ready";
    }
    if (!positionFresh(now())) {
      return "Pixhawk local position is not fresh";
    }
    if (require_start_frame && !start_frame_ready_) {
      return "start frame is not ready";
    }
    if (!have_current_yaw_) {
      return "Pixhawk local orientation is not valid";
    }

    const double goal_distance = distance(
      requested, current_odometry_.pose.pose.position);
    if (max_goal_distance_m_ > 0.0 && goal_distance > max_goal_distance_m_) {
      return
        "goal is farther than max_goal_distance_m (" +
        std::to_string(goal_distance) + " m)";
    }
    if (requested.z < min_goal_z_m_ || requested.z > max_goal_z_m_) {
      return
        "goal z is outside configured limits [" +
        std::to_string(min_goal_z_m_) + ", " +
        std::to_string(max_goal_z_m_) + "]";
    }
    return std::nullopt;
  }

  static std::string normalizedFrameId(const std::string & frame_id)
  {
    const auto first = frame_id.find_first_not_of('/');
    return first == std::string::npos ? std::string() : frame_id.substr(first);
  }

  void rejectGoal(const std::string & reason)
  {
    setStatus("REJECTED: " + reason);
    RCLCPP_WARN(get_logger(), "Guided goal rejected: %s", reason.c_str());
  }

  void cancelCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    if (control_state_ == ControlState::IDLE) {
      return;
    }
    stopGoal("IDLE: goal cancelled; no further setpoints will be published");
    RCLCPP_WARN(get_logger(), "Guided goal cancelled");
  }

  void stopGoal(const std::string & status)
  {
    control_state_ = ControlState::IDLE;
    goal_source_ = GoalSource::NONE;
    mode_request_pending_ = false;
    arrival_started_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    heading_aligned_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    publishArrived(false);
    publishMissionStatus(auv_msg::msg::MissionStatus::READY);
    setStatus(status);
  }

  void recaptureStartFrameCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    if (control_state_ != ControlState::IDLE) {
      RCLCPP_WARN(
        get_logger(),
        "Start-frame recapture rejected: a guided goal is active");
      return;
    }
    if (start_frame_require_disarmed_ && have_state_ && fcu_armed_) {
      RCLCPP_WARN(
        get_logger(),
        "Start-frame recapture rejected: FCU must be disarmed");
      return;
    }

    start_frame_ready_ = false;
    start_frame_capture_pending_ = true;
    tryCaptureStartFrame();
    if (!start_frame_ready_) {
      RCLCPP_WARN(
        get_logger(),
        "Start-frame recapture pending: waiting for ExternalNav and fresh odometry");
    }
  }

  void localPositionCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    if (!isFinite(position.x) || !isFinite(position.y) || !isFinite(position.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-finite Pixhawk local position");
      return;
    }
    current_odometry_ = *message;
    have_position_ = true;
    last_position_rx_ = now();

    const auto yaw = yawFromQuaternion(message->pose.pose.orientation);
    if (yaw.has_value()) {
      current_yaw_ = *yaw;
      have_current_yaw_ = true;
    } else {
      have_current_yaw_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Pixhawk local orientation is invalid");
    }
    tryCaptureStartFrame();
    tryActivatePendingWaypoint();
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr message)
  {
    have_state_ = true;
    fcu_connected_ = message->connected;
    fcu_armed_ = message->armed;
    current_mode_ = message->mode;
    tryCaptureStartFrame();

    if (
      control_state_ != ControlState::IDLE &&
      !fcu_armed_ && message->connected)
    {
      stopGoal("IDLE: FCU was disarmed; guided goal cleared");
    }
    tryActivatePendingWaypoint();
  }

  void externalNavReadyCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    const bool was_ready = external_nav_ready_;
    external_nav_ready_ = message->data;
    if (!external_nav_ready_) {
      start_frame_ready_ = false;
      start_frame_capture_pending_ = false;
      return;
    }
    if (!was_ready || !start_frame_ready_) {
      start_frame_capture_pending_ = true;
      tryCaptureStartFrame();
    }
    tryActivatePendingWaypoint();
  }

  void tryCaptureStartFrame()
  {
    if (
      !start_frame_capture_pending_ || !external_nav_ready_ ||
      !positionFresh(now()) || !have_current_yaw_)
    {
      return;
    }
    if (
      start_frame_require_disarmed_ &&
      (!have_state_ || fcu_armed_))
    {
      return;
    }

    start_frame_origin_ = current_odometry_.pose.pose.position;
    start_frame_yaw_ = current_yaw_;
    start_frame_ready_ = true;
    start_frame_capture_pending_ = false;
    publishStartFrame();

    RCLCPP_INFO(
      get_logger(),
      "Captured guided start frame: odom origin=(%.3f, %.3f, %.3f), "
      "start yaw=%.2f deg; this direction is guided yaw 0 deg",
      start_frame_origin_.x, start_frame_origin_.y, start_frame_origin_.z,
      start_frame_yaw_ * 180.0 / 3.14159265358979323846);
  }

  void controlLoop()
  {
    if (control_state_ == ControlState::IDLE) {
      return;
    }

    const auto current_time = now();
    const bool position_fresh = positionFresh(current_time);

    if (control_state_ == ControlState::PRIMING) {
      publishSetpoint(
        hold_position_, target_yaw_enabled_,
        target_yaw_enabled_ ? hold_yaw_ : 0.0);
      if ((current_time - phase_started_at_).seconds() >= prime_duration_s_) {
        control_state_ = ControlState::WAITING_GUIDED;
        setStatus("WAITING_GUIDED: requesting GUIDED mode");
        requestGuidedMode();
      }
      return;
    }

    if (control_state_ == ControlState::WAITING_GUIDED) {
      publishSetpoint(
        hold_position_, target_yaw_enabled_,
        target_yaw_enabled_ ? hold_yaw_ : 0.0);
      if (current_mode_ == guided_mode_) {
        if (target_yaw_enabled_ && align_heading_before_move_) {
          control_state_ = ControlState::ALIGNING_HEADING;
          phase_started_at_ = current_time;
          heading_aligned_at_ = rclcpp::Time(
            0, 0, get_clock()->get_clock_type());
          setStatus("ALIGNING_HEADING: rotating in place to face target");
        } else {
          control_state_ = ControlState::MOVING;
          setStatus("MOVING: target " + pointText(target_position_));
        }
      } else {
        requestGuidedMode();
      }
      return;
    }

    if (control_state_ != ControlState::HOLDING) {
      updateTargetHeading(true);
    }

    if (control_state_ == ControlState::ALIGNING_HEADING) {
      publishSetpoint(hold_position_, true, target_yaw_);
    } else {
      publishSetpoint(target_position_, target_yaw_enabled_, target_yaw_);
    }

    if (
      control_state_ == ControlState::ALIGNING_HEADING &&
      heading_timeout_s_ > 0.0 &&
      (current_time - phase_started_at_).seconds() >= heading_timeout_s_)
    {
      control_state_ = ControlState::MOVING;
      heading_aligned_at_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      setStatus(
        "MOVING: heading alignment timeout; automatic target heading continues");
      return;
    }

    if (current_mode_ != guided_mode_) {
      if (restore_guided_mode_) {
        requestGuidedMode();
        setStatus("DEGRADED: GUIDED mode lost; requesting it again");
      } else {
        setStatus("DEGRADED: GUIDED mode lost; target streaming continues");
      }
      return;
    }

    if (!fcu_connected_) {
      setStatus("DEGRADED: FCU disconnected; target streaming continues");
      return;
    }
    if (!position_fresh) {
      setStatus("DEGRADED: Pixhawk local position is stale; target streaming continues");
      return;
    }

    if (control_state_ == ControlState::ALIGNING_HEADING) {
      const double yaw_error =
        std::abs(normalizeAngle(target_yaw_ - current_yaw_));
      if (!have_current_yaw_ || yaw_error > heading_tolerance_rad_) {
        heading_aligned_at_ = rclcpp::Time(
          0, 0, get_clock()->get_clock_type());
        setProgressStatus(
          current_time,
          "ALIGNING_HEADING: yaw_error=" +
          std::to_string(
            yaw_error * 180.0 / 3.14159265358979323846) +
          " deg");
        return;
      }

      if (heading_aligned_at_.nanoseconds() == 0) {
        heading_aligned_at_ = current_time;
        setStatus("ALIGNING_HEADING: inside yaw tolerance; settling");
        return;
      }
      if (
        (current_time - heading_aligned_at_).seconds() <
        heading_settle_time_s_)
      {
        return;
      }

      control_state_ = ControlState::MOVING;
      setStatus("MOVING: heading aligned; target " + pointText(target_position_));
      return;
    }

    if (control_state_ == ControlState::HOLDING) {
      setStatus("HOLDING: arrived at target " + pointText(target_position_));
      return;
    }

    const double remaining = distance(
      target_position_, current_odometry_.pose.pose.position);
    const double current_speed = speed(current_odometry_);
    const double yaw_error =
      target_yaw_enabled_ ?
      std::abs(normalizeAngle(target_yaw_ - current_yaw_)) : 0.0;
    const bool yaw_arrived =
      !target_yaw_enabled_ ||
      (have_current_yaw_ && yaw_error <= arrival_yaw_tolerance_rad_);
    const bool inside_arrival_window =
      remaining <= arrival_radius_m_ &&
      current_speed <= arrival_speed_mps_ &&
      yaw_arrived;

    if (!inside_arrival_window) {
      arrival_started_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      setProgressStatus(
        current_time,
        "MOVING: distance=" + std::to_string(remaining) +
        " m speed=" + std::to_string(current_speed) +
        " m/s yaw_error=" +
        std::to_string(yaw_error * 180.0 / 3.14159265358979323846) +
        " deg");
      return;
    }

    if (arrival_started_at_.nanoseconds() == 0) {
      arrival_started_at_ = current_time;
    }
    if ((current_time - arrival_started_at_).seconds() < arrival_settle_time_s_) {
      setStatus("SETTLING: inside arrival radius");
      return;
    }

    control_state_ = ControlState::HOLDING;
    publishArrived(true);
    publishMissionStatus(auv_msg::msg::MissionStatus::COMPLETED);
    setStatus("HOLDING: arrived at target " + pointText(target_position_));
    RCLCPP_INFO(
      get_logger(),
      "Arrived: target=(%.3f, %.3f, %.3f), remaining=%.3f m, speed=%.3f m/s",
      target_position_.x, target_position_.y, target_position_.z,
      remaining, current_speed);
  }

  void publishSetpoint(
    const geometry_msgs::msg::Point & position,
    bool use_yaw,
    double yaw)
  {
    mavros_msgs::msg::PositionTarget target;
    target.header.stamp = now();
    target.header.frame_id = command_frame_;
    target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_VX |
      mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    if (!use_yaw) {
      target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
    } else {
      target.yaw = yaw;
    }
    target.position = position;
    setpoint_pub_->publish(target);
  }

  void requestGuidedMode()
  {
    if (!fcu_connected_ || !fcu_armed_ || mode_request_pending_) {
      return;
    }

    const auto steady_time = std::chrono::steady_clock::now();
    if (last_mode_request_.time_since_epoch().count() != 0) {
      const double elapsed =
        std::chrono::duration<double>(steady_time - last_mode_request_).count();
      if (elapsed < mode_request_interval_s_) {
        return;
      }
    }
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SetMode service %s is not ready", set_mode_service_.c_str());
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = guided_mode_;
    last_mode_request_ = steady_time;
    mode_request_pending_ = true;

    set_mode_client_->async_send_request(
      request,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        mode_request_pending_ = false;
        const auto response = future.get();
        if (!response->mode_sent) {
          RCLCPP_WARN(get_logger(), "GUIDED mode request was rejected");
        } else {
          RCLCPP_INFO(get_logger(), "GUIDED mode request sent");
        }
      });
  }

  bool positionFresh(const rclcpp::Time & current_time) const
  {
    return have_position_ &&
           last_position_rx_.nanoseconds() != 0 &&
           (current_time - last_position_rx_).seconds() <= position_timeout_s_;
  }

  void publishArrived(bool arrived)
  {
    std_msgs::msg::Bool message;
    message.data = arrived;
    arrived_pub_->publish(message);
  }

  void publishMissionStatus(int8_t status)
  {
    auv_msg::msg::MissionStatus message;
    message.status = status;
    mission_status_pub_->publish(message);
  }

  void updateTargetHeading(bool publish_update)
  {
    const double dx =
      target_position_.x - current_odometry_.pose.pose.position.x;
    const double dy =
      target_position_.y - current_odometry_.pose.pose.position.y;
    if (std::hypot(dx, dy) <= heading_update_min_distance_m_) {
      return;
    }

    const double next_yaw = std::atan2(dy, dx);
    const double yaw_change =
      std::abs(normalizeAngle(next_yaw - target_yaw_));
    target_yaw_ = next_yaw;

    constexpr double active_target_update_threshold_rad =
      0.0087266463;  // 0.5 deg
    if (publish_update && yaw_change >= active_target_update_threshold_rad) {
      publishActiveTarget();
    }
  }

  void publishActiveTarget()
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = command_frame_;
    message.pose.position = target_position_;
    if (target_yaw_enabled_) {
      message.pose.orientation = quaternionFromYaw(target_yaw_);
    } else {
      message.pose.orientation.w = 1.0;
    }
    active_target_pub_->publish(message);
  }

  void publishStartFrame()
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = command_frame_;
    message.pose.position = start_frame_origin_;
    message.pose.orientation = quaternionFromYaw(start_frame_yaw_);
    start_frame_pub_->publish(message);
  }

  void setStatus(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", status.c_str());
  }

  void setProgressStatus(
    const rclcpp::Time & current_time,
    const std::string & status)
  {
    if (
      last_progress_status_at_.nanoseconds() != 0 &&
      (current_time - last_progress_status_at_).seconds() <
      progress_status_interval_s_)
    {
      return;
    }
    last_progress_status_at_ = current_time;
    setStatus(status);
  }

  static std::string pointText(const geometry_msgs::msg::Point & point)
  {
    return "(" + std::to_string(point.x) + ", " +
           std::to_string(point.y) + ", " +
           std::to_string(point.z) + ")";
  }

  static std::string goalModeText(int32_t mode)
  {
    if (mode == 0) {
      return "absolute odom";
    }
    if (mode == 1) {
      return "odom-axis relative";
    }
    return "start-frame absolute";
  }

  std::string goal_topic_;
  std::string waypoint_topic_;
  std::string waypoint_enable_topic_;
  std::string cancel_topic_;
  std::string local_position_topic_;
  std::string state_topic_;
  std::string external_nav_ready_topic_;
  std::string recapture_start_frame_topic_;
  std::string setpoint_topic_;
  std::string set_mode_service_;
  std::string status_topic_;
  std::string arrived_topic_;
  std::string mission_status_topic_;
  std::string active_target_topic_;
  std::string start_frame_topic_;
  std::string command_frame_;
  std::string guided_mode_;

  double setpoint_rate_hz_{10.0};
  double position_timeout_s_{0.5};
  double prime_duration_s_{0.5};
  double mode_request_interval_s_{1.0};
  double progress_status_interval_s_{1.0};
  double arrival_radius_m_{0.2};
  double arrival_speed_mps_{0.1};
  double arrival_settle_time_s_{1.0};
  double arrival_yaw_tolerance_rad_{0.1745329252};
  double heading_tolerance_rad_{0.0872664626};
  double heading_settle_time_s_{0.5};
  double heading_timeout_s_{5.0};
  double heading_update_min_distance_m_{0.2};
  double max_goal_distance_m_{20.0};
  double min_goal_z_m_{-50.0};
  double max_goal_z_m_{1.0};
  double waypoint_update_epsilon_m_{0.02};
  double waypoint_cache_timeout_s_{1.0};
  bool restore_guided_mode_{true};
  bool start_frame_require_disarmed_{true};
  bool align_heading_before_move_{true};

  ControlState control_state_{ControlState::IDLE};
  bool have_state_{false};
  bool have_position_{false};
  bool fcu_connected_{false};
  bool fcu_armed_{false};
  bool external_nav_ready_{false};
  bool mode_request_pending_{false};
  bool have_current_yaw_{false};
  bool start_frame_ready_{false};
  bool start_frame_capture_pending_{false};
  bool target_yaw_enabled_{false};
  bool waypoint_enabled_{false};
  GoalSource goal_source_{GoalSource::NONE};
  std::string current_mode_;
  std::string last_status_;
  geometry_msgs::msg::Point hold_position_;
  geometry_msgs::msg::Point target_position_;
  geometry_msgs::msg::Point start_frame_origin_;
  double current_yaw_{0.0};
  double hold_yaw_{0.0};
  double target_yaw_{0.0};
  double start_frame_yaw_{0.0};
  nav_msgs::msg::Odometry current_odometry_;
  std::optional<mavros_msgs::msg::PositionTarget> pending_waypoint_;
  rclcpp::Time last_position_rx_;
  rclcpp::Time last_waypoint_rx_;
  rclcpp::Time phase_started_at_;
  rclcpp::Time arrival_started_at_;
  rclcpp::Time heading_aligned_at_;
  rclcpp::Time last_progress_status_at_;
  std::chrono::steady_clock::time_point last_mode_request_;

  rclcpp::Subscription<auv_msg::msg::AuvSetpoint>::SharedPtr goal_sub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recapture_start_frame_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr external_nav_ready_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrived_pub_;
  rclcpp::Publisher<auv_msg::msg::MissionStatus>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr active_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_frame_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidedNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
