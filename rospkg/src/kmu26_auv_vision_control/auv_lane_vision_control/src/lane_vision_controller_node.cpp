#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "auv_lane_vision_control/arena_frame_transform.hpp"
#include "auv_lane_vision_control/lane_planner.hpp"

namespace auv_lane_vision_control
{
class LaneVisionControllerNode : public rclcpp::Node
{
public:
  // 파라미터와 통신 인터페이스를 준비하고 제어 노드를 초기화한다.
  LaneVisionControllerNode()
  : Node("lane_vision_controller_node")
  {
    declare_topics();
    declare_arena_parameters();
    declare_mission_parameters();
    declare_control_parameters();
    validate_parameters();

    lane_planner_ = std::make_unique<LanePlanner>(
      ArenaConfig{
        arena_length_m_,
        arena_width_m_,
        arena_offset_x_m_,
        arena_offset_y_m_,
        arena_safety_margin_m_,
        lane_search_offset_m_,
        arena_start_corner_});
    lane_completed_.assign(lane_planner_->lanes().size(), false);

    create_interfaces();

    state_entered_at_ = now();
    publish_state();
    publish_target_confirmed(false);
    publish_success(false);
    publish_lane_event("ready");

    const auto & bounds = lane_planner_->safe_bounds();
    RCLCPP_INFO(
      get_logger(),
      "Lane vision controller ready: lanes=%zu spacing=%.2f m "
      "safe_bounds=[%.2f, %.2f]x[%.2f, %.2f]",
      lane_planner_->lanes().size(), lane_planner_->actual_lane_spacing_m(),
      bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max);
    if (
      expected_lane_count_ > 0 &&
      lane_planner_->lanes().size() != static_cast<std::size_t>(expected_lane_count_))
    {
      throw std::invalid_argument(
              "generated lane count does not match expected_lane_count");
    }
    RCLCPP_INFO(
      get_logger(),
      "RC remains silent until %s grants control",
      vision_control_granted_topic_.c_str());
  }

  // 노드 종료 전에 사용한 제어 채널을 한 번 해제한다.
  void publish_release_once()
  {
    if (!vision_has_control_) {
      return;
    }
    auto channels = nochange_channels();
    release_controlled_channels(channels);
    publish_channels(channels);
  }

private:
  enum class State
  {
    IDLE,
    TARGET_CONFIRM,
    WAIT_CONTROL_GRANT,
    INITIAL_SCAN_360,
    TURN_TO_INITIAL_TARGET,
    FIND_AND_ALIGN,
    ALIGN_STICK,
    INSERT_FORK,
    DETACH,
    GO_BACK,
    VERIFY_RELEASE,
    MOVE_TO_LANE_START,
    LANE_FOLLOWING_WITH_SEARCH,
    RETURN_TO_ACTIVE_LANE,
    COMPLETE,
    FAILSAFE
  };

  enum class TargetContext
  {
    INITIAL,
    LANE
  };

  struct Detection
  {
    float confidence{0.0F};
    float center_x{0.0F};
    float center_y{0.0F};
    float width{0.0F};
    float height{0.0F};
    float image_width{0.0F};
    float image_height{0.0F};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    int consecutive_hits{1};
  };

  struct DepthSample
  {
    double depth_m{0.0};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  };

  struct ScanCandidate
  {
    Detection detection{};
    double arena_yaw_rad{0.0};
    double area_ratio{0.0};
  };

  // 입력과 출력에 사용할 토픽 이름 파라미터를 선언한다.
  void declare_topics()
  {
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/vision/buoy_bbox");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/auv/depth");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    odometry_topic_ =
      declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
    start_frame_topic_ =
      declare_parameter<std::string>("start_frame_topic", "/start_frame");
    vision_search_request_topic_ = declare_parameter<std::string>(
      "vision_search_request_topic", "/homing/vision_search_active");
    target_confirmed_topic_ =
      declare_parameter<std::string>("target_confirmed_topic", "/vision/target_confirmed");
    vision_control_granted_topic_ = declare_parameter<std::string>(
      "vision_control_granted_topic", "/homing/vision_control_granted");
    emergency_stop_topic_ =
      declare_parameter<std::string>("emergency_stop_topic", "/mission/emergency_stop");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mission/state");
    rc_override_topic_ =
      declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
    rc_monitor_topic_ =
      declare_parameter<std::string>("rc_monitor_topic", "/mission/rc_command");
    fcu_state_topic_ =
      declare_parameter<std::string>("fcu_state_topic", "/mavros/state");
    physical_detach_count_topic_ = declare_parameter<std::string>(
      "physical_detach_count_topic", "/vision/course_buoy_detach_count");
    lane_event_topic_ =
      declare_parameter<std::string>("lane_event_topic", "/mission/lane_event");
    success_topic_ =
      declare_parameter<std::string>("success_topic", "/mission/success");
    required_fcu_mode_ =
      declare_parameter<std::string>("required_fcu_mode", "");
  }

  // 수조 경계와 레인 생성에 필요한 파라미터를 선언한다.
  void declare_arena_parameters()
  {
    arena_length_m_ = declare_parameter<double>("arena_length_m", 15.0);
    arena_width_m_ = declare_parameter<double>("arena_width_m", 16.0);
    arena_offset_x_m_ = declare_parameter<double>("arena_offset_x_m", -0.3);
    arena_offset_y_m_ = declare_parameter<double>("arena_offset_y_m", 0.3);
    arena_safety_margin_m_ = declare_parameter<double>("arena_safety_margin_m", 0.5);
    arena_start_corner_ =
      declare_parameter<std::string>("arena_start_corner", "bottom_left");
    lane_search_offset_m_ = declare_parameter<double>("lane_search_offset_m", 2.0);
    incapable_skip_distance_m_ =
      declare_parameter<double>("incapable_skip_distance_m", 2.0);
    waypoint_reach_tolerance_m_ =
      declare_parameter<double>("waypoint_reach_tolerance_m", 0.15);
    expected_lane_count_ = declare_parameter<int>("expected_lane_count", 0);
  }

  // 탐색과 부표 작업에 필요한 파라미터를 선언한다.
  void declare_mission_parameters()
  {
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    odometry_timeout_sec_ = declare_parameter<double>("odometry_timeout_sec", 0.5);
    detection_timeout_sec_ = declare_parameter<double>("detection_timeout_sec", 0.7);
    depth_timeout_sec_ = declare_parameter<double>("depth_timeout_sec", 3.0);
    depth_pose_scale_ = declare_parameter<double>("depth_pose_scale", -1.0);
    depth_pose_offset_m_ = declare_parameter<double>("depth_pose_offset_m", 0.0);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10.5);

    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    stick_class_id_ = declare_parameter<int>("stick_class_id", 1);
    target_confirm_hits_ = declare_parameter<int>("target_confirm_hits", 4);
    target_confirm_sec_ = declare_parameter<double>("target_confirm_sec", 0.3);
    same_target_center_ratio_ =
      declare_parameter<double>("same_target_center_ratio", 0.12);
    lpf_tau_sec_ = declare_parameter<double>("lpf_tau_sec", 0.3);

    initial_search_radius_m_ =
      declare_parameter<double>("initial_search_radius_m", 2.0);
    initial_scan_yaw_pwm_ = declare_parameter<int>("initial_scan_yaw_pwm", 1600);
    initial_scan_completion_tolerance_rad_ =
      declare_parameter<double>("initial_scan_completion_tolerance_rad", 0.15);
    initial_target_reacquire_timeout_sec_ =
      declare_parameter<double>("initial_target_reacquire_timeout_sec", 5.0);
    target_reacquire_timeout_sec_ =
      declare_parameter<double>("target_reacquire_timeout_sec", 2.0);

    align_target_x_ = declare_parameter<double>("align_target_x", 0.50);
    align_target_y_ = declare_parameter<double>("align_target_y", 0.50);
    align_deadband_x_ = declare_parameter<double>("align_deadband_x", 0.08);
    align_deadband_y_ = declare_parameter<double>("align_deadband_y", 0.10);
    approach_area_ratio_ =
      declare_parameter<double>("approach_area_ratio", 0.0035);
    approach_forward_min_pwm_ =
      declare_parameter<int>("approach_forward_min_pwm", 1540);

    fork_target_x_ = declare_parameter<double>("fork_target_x", 0.44);
    fork_target_y_ = declare_parameter<double>("fork_target_y", 0.54);
    lane_fork_target_y_ =
      declare_parameter<double>("lane_fork_target_y", fork_target_y_);
    lane_fork_depth_offset_m_ =
      declare_parameter<double>("lane_fork_depth_offset_m", 0.0);
    stick_deadband_x_ = declare_parameter<double>("stick_deadband_x", 0.04);
    stick_deadband_y_ = declare_parameter<double>("stick_deadband_y", 0.08);
    align_stable_sec_ = declare_parameter<double>("align_stable_sec", 0.3);

    insert_fork_pwm_ = declare_parameter<int>("insert_fork_pwm", 1700);
    insert_fork_duration_sec_ =
      declare_parameter<double>("insert_fork_duration_sec", 2.0);
    detach_pwm_ = declare_parameter<int>("detach_pwm", 1700);
    detach_duration_sec_ =
      declare_parameter<double>("detach_duration_sec", 1.0);
    go_back_pwm_ = declare_parameter<int>("go_back_pwm", 1350);
    go_back_duration_sec_ = declare_parameter<double>("go_back_duration_sec", 1.0);
    verify_clear_sec_ = declare_parameter<double>("verify_clear_sec", 1.0);
    verify_timeout_sec_ = declare_parameter<double>("verify_timeout_sec", 3.0);
    max_target_retries_ = declare_parameter<int>("max_target_retries", 2);
    require_physical_detach_ =
      declare_parameter<bool>("require_physical_detach", false);
  }

  // 이동과 수심 유지에 필요한 제어 파라미터를 선언한다.
  void declare_control_parameters()
  {
    throttle_channel_ = declare_parameter<int>("throttle_channel", 3);
    yaw_channel_ = declare_parameter<int>("yaw_channel", 4);
    forward_channel_ = declare_parameter<int>("forward_channel", 5);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    rc_pwm_span_ = declare_parameter<double>("rc_pwm_span", 400.0);

    lane_forward_pwm_ = declare_parameter<int>("lane_forward_pwm", 1700);
    approach_forward_pwm_ = declare_parameter<int>("approach_forward_pwm", 1560);
    waypoint_heading_tolerance_rad_ =
      declare_parameter<double>("waypoint_heading_tolerance_rad", 0.1745);
    waypoint_yaw_kp_ = declare_parameter<double>("waypoint_yaw_kp", 1.15);
    waypoint_yaw_ki_ = declare_parameter<double>("waypoint_yaw_ki", 0.15);
    waypoint_yaw_kd_ = declare_parameter<double>("waypoint_yaw_kd", 0.08);
    waypoint_yaw_integral_limit_ =
      declare_parameter<double>("waypoint_yaw_integral_limit", 2.0);
    max_waypoint_yaw_delta_pwm_ =
      declare_parameter<int>("max_waypoint_yaw_delta_pwm", 180);
    waypoint_yaw_invert_ = declare_parameter<bool>("waypoint_yaw_invert", true);

    max_vision_yaw_delta_pwm_ =
      declare_parameter<int>("max_vision_yaw_delta_pwm", 180);
    max_vision_throttle_delta_pwm_ =
      declare_parameter<int>("max_vision_throttle_delta_pwm", 250);
    min_effective_yaw_delta_pwm_ =
      declare_parameter<int>("min_effective_yaw_delta_pwm", 0);
    min_effective_vertical_delta_pwm_ =
      declare_parameter<int>("min_effective_vertical_delta_pwm", 0);
    vision_yaw_invert_ = declare_parameter<bool>("vision_yaw_invert", false);
    vertical_positive_is_up_ =
      declare_parameter<bool>("vertical_positive_is_up", true);
    approach_vision_throttle_weight_ =
      declare_parameter<double>("approach_vision_throttle_weight", 0.4);
    vertical_full_weight_error_ =
      declare_parameter<double>("vertical_full_weight_error", 0.25);

    depth_kp_pwm_per_m_ = declare_parameter<double>("depth_kp_pwm_per_m", 120.0);
    depth_ki_pwm_per_m_sec_ =
      declare_parameter<double>("depth_ki_pwm_per_m_sec", 20.0);
    depth_kd_pwm_sec_per_m_ =
      declare_parameter<double>("depth_kd_pwm_sec_per_m", 16.0);
    depth_integral_limit_m_sec_ =
      declare_parameter<double>("depth_integral_limit_m_sec", 2.0);
    buoyancy_hold_delta_pwm_ =
      declare_parameter<int>("buoyancy_hold_delta_pwm", 40);
    max_depth_delta_pwm_ = declare_parameter<int>("max_depth_delta_pwm", 160);
    depth_deadband_m_ = declare_parameter<double>("depth_deadband_m", 0.04);
  }

  // 선언한 파라미터가 안전한 범위와 관계를 만족하는지 확인한다.
  void validate_parameters() const
  {
    if (
      min_pwm_ < 1300 || max_pwm_ > 1700 || min_pwm_ >= max_pwm_ ||
      neutral_pwm_ < min_pwm_ || neutral_pwm_ > max_pwm_)
    {
      throw std::invalid_argument(
              "PWM range must stay within 1300..1700 and include neutral_pwm");
    }
    for (const int channel : {throttle_channel_, yaw_channel_, forward_channel_}) {
      if (channel < 1 || channel > 18) {
        throw std::invalid_argument("controlled RC channels must be in [1, 18]");
      }
    }
    if (
      throttle_channel_ == yaw_channel_ || throttle_channel_ == forward_channel_ ||
      yaw_channel_ == forward_channel_)
    {
      throw std::invalid_argument("controlled RC channels must be unique");
    }
    if (
      control_rate_hz_ <= 0.0 || odometry_timeout_sec_ <= 0.0 ||
      detection_timeout_sec_ <= 0.0 || depth_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("control rate and input timeouts must be positive");
    }
    if (
      target_confirm_hits_ < 1 || target_confirm_sec_ < 0.0 ||
      lpf_tau_sec_ < 0.0 || initial_search_radius_m_ <= 0.0 ||
      target_reacquire_timeout_sec_ < 0.0)
    {
      throw std::invalid_argument("detection and initial search parameters are invalid");
    }
    if (
      align_target_x_ < 0.0 || align_target_x_ > 1.0 ||
      align_target_y_ < 0.0 || align_target_y_ > 1.0 ||
      fork_target_x_ < 0.0 || fork_target_x_ > 1.0 ||
      fork_target_y_ < 0.0 || fork_target_y_ > 1.0 ||
      lane_fork_target_y_ < 0.0 || lane_fork_target_y_ > 1.0 ||
      std::abs(lane_fork_depth_offset_m_) > 1.0 ||
      approach_area_ratio_ <= 0.0 || approach_area_ratio_ > 1.0)
    {
      throw std::invalid_argument("image alignment parameters must be normalized");
    }
    if (
      approach_vision_throttle_weight_ < 0.0 ||
      approach_vision_throttle_weight_ > 1.0 ||
      vertical_full_weight_error_ <= 0.0 ||
      vertical_full_weight_error_ > 1.0)
    {
      throw std::invalid_argument(
              "vision throttle weight must be in [0, 1] and "
              "vertical_full_weight_error in (0, 1]");
    }
    if (
      insert_fork_duration_sec_ < 0.0 || detach_duration_sec_ < 0.0 ||
      go_back_duration_sec_ < 0.0 || verify_clear_sec_ < 0.0 ||
      verify_timeout_sec_ < verify_clear_sec_ || max_target_retries_ < 0)
    {
      throw std::invalid_argument("fork and verification durations are invalid");
    }
    if (buoy_class_id_ == stick_class_id_) {
      throw std::invalid_argument("buoy_class_id and stick_class_id must differ");
    }
    const int pwm_headroom = std::min(max_pwm_ - neutral_pwm_, neutral_pwm_ - min_pwm_);
    if (
      min_effective_yaw_delta_pwm_ < 0 ||
      min_effective_yaw_delta_pwm_ > std::min(max_vision_yaw_delta_pwm_, pwm_headroom) ||
      min_effective_vertical_delta_pwm_ < 0 ||
      min_effective_vertical_delta_pwm_ >
      std::min(max_vision_throttle_delta_pwm_, pwm_headroom))
    {
      throw std::invalid_argument("visual deadzone compensation exceeds PWM range");
    }
  }

  // 구독자와 발행자 및 주기 실행기를 생성한다.
  void create_interfaces()
  {
    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, 10,
      std::bind(&LaneVisionControllerNode::on_bbox, this, std::placeholders::_1));
    depth_sub_ = create_subscription<std_msgs::msg::Float64>(
      depth_topic_, 10,
      std::bind(&LaneVisionControllerNode::on_depth, this, std::placeholders::_1));
    if (!depth_pose_topic_.empty()) {
      depth_pose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        depth_pose_topic_, 10,
        std::bind(&LaneVisionControllerNode::on_depth_pose, this, std::placeholders::_1));
    }
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 30,
      std::bind(&LaneVisionControllerNode::on_odometry, this, std::placeholders::_1));
    start_frame_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      start_frame_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LaneVisionControllerNode::on_start_frame, this, std::placeholders::_1));
    vision_search_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_search_request_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &LaneVisionControllerNode::on_vision_search_request, this,
        std::placeholders::_1));
    vision_control_granted_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_control_granted_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &LaneVisionControllerNode::on_vision_control_granted, this,
        std::placeholders::_1));
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LaneVisionControllerNode::on_emergency_stop, this, std::placeholders::_1));
    physical_detach_count_sub_ = create_subscription<std_msgs::msg::UInt32>(
      physical_detach_count_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &LaneVisionControllerNode::on_physical_detach_count, this,
        std::placeholders::_1));
    fcu_state_sub_ = create_subscription<mavros_msgs::msg::State>(
      fcu_state_topic_, 10,
      std::bind(
        &LaneVisionControllerNode::on_fcu_state, this,
        std::placeholders::_1));

    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
    rc_monitor_pub_ =
      create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_monitor_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());
    target_confirmed_pub_ = create_publisher<std_msgs::msg::Bool>(
      target_confirmed_topic_, rclcpp::QoS(1).reliable().transient_local());
    lane_event_pub_ = create_publisher<std_msgs::msg::String>(
      lane_event_topic_, rclcpp::QoS(10).reliable().transient_local());
    success_pub_ = create_publisher<std_msgs::msg::Bool>(
      success_topic_, rclcpp::QoS(1).reliable().transient_local());

    const double period_sec = 1.0 / control_rate_hz_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&LaneVisionControllerNode::on_timer, this));
  }

  // 시작 위치와 방향을 받아 수조 좌표 변환 기준을 설정한다.
  void on_start_frame(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (arena_transform_.initialized()) {
      return;
    }
    const Vec2 origin{msg->pose.position.x, msg->pose.position.y};
    const double yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
    if (!finite(origin) || !std::isfinite(yaw)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid start frame");
      return;
    }
    arena_transform_.initialize(origin, yaw);
    RCLCPP_INFO(
      get_logger(), "Arena start frame accepted: origin=(%.2f, %.2f), yaw=%.3f",
      origin.x, origin.y, yaw);
  }

  // 위치와 방향을 수조 좌표로 변환해 현재 상태를 갱신한다.
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Vec2 odom_position{msg->pose.pose.position.x, msg->pose.pose.position.y};
    const double odom_yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (!finite(odom_position) || !std::isfinite(odom_yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-finite odometry");
      return;
    }
    if (!arena_transform_.initialized()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for /start_frame");
      return;
    }

    current_position_ = arena_transform_.position_from_odom(odom_position);
    current_yaw_rad_ = arena_transform_.yaw_from_odom(odom_yaw);
    odometry_received_at_ = now();
    have_odometry_ = true;
  }

  // 단일 수심 값을 보조 수심 입력으로 저장한다.
  void on_depth(const std_msgs::msg::Float64::SharedPtr msg)
  {
    accept_depth(scalar_depth_, msg->data);
  }

  // 위치 메시지의 높이값을 주 수심 입력으로 변환해 저장한다.
  void on_depth_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    accept_depth(
      pose_depth_,
      depth_pose_scale_ * msg->pose.pose.position.z + depth_pose_offset_m_);
  }

  // 수심 값의 유효성을 검사한 뒤 수신 시각과 함께 저장한다.
  void accept_depth(std::optional<DepthSample> & slot, const double depth_m)
  {
    if (!std::isfinite(depth_m) || depth_m < 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring invalid depth sample");
      return;
    }
    slot = DepthSample{depth_m, now()};
  }

  // 검출 메시지에서 가장 적합한 부표 상자를 선택해 갱신한다.
  void on_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 10) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring bbox with fewer than 10 values");
      return;
    }

    std::optional<Detection> best_buoy;
    std::optional<Detection> best_stick;
    const size_t block_count = msg->data.size() / 10;
    for (size_t block = 0; block < block_count; ++block) {
      const size_t base = block * 10;
      bool values_are_finite = true;
      for (size_t index = 0; index < 10; ++index) {
        if (!std::isfinite(msg->data[base + index])) {
          values_are_finite = false;
          break;
        }
      }
      if (
        !values_are_finite || msg->data[base + 1] < 0.5F ||
        msg->data[base + 8] <= 0.0F || msg->data[base + 9] <= 0.0F)
      {
        continue;
      }

      const int class_id = static_cast<int>(std::lround(msg->data[base + 2]));
      if (class_id != buoy_class_id_ && class_id != stick_class_id_) {
        continue;
      }
      Detection candidate{
        msg->data[base + 3],
        msg->data[base + 4],
        msg->data[base + 5],
        msg->data[base + 6],
        msg->data[base + 7],
        msg->data[base + 8],
        msg->data[base + 9],
        now(),
        1};
      if (class_id == buoy_class_id_) {
        if (!best_buoy || is_better_buoy(candidate, *best_buoy)) {
          best_buoy = candidate;
        }
      } else if (!best_stick || is_better_buoy(candidate, *best_stick)) {
        best_stick = candidate;
      }
    }

    if (best_buoy) {
      const Detection raw_detection = *best_buoy;
      update_detection_slot(buoy_, raw_detection);
      if (state_ == State::INITIAL_SCAN_360 && have_odometry_) {
        consider_scan_candidate(raw_detection);
      }
    }
    if (best_stick) {
      update_detection_slot(stick_, *best_stick);
    }
  }

  // 상자 면적과 신뢰도를 비교해 더 가까운 부표 후보를 판단한다.
  bool is_better_buoy(const Detection & candidate, const Detection & current) const
  {
    const double candidate_area =
      static_cast<double>(candidate.width) * static_cast<double>(candidate.height);
    const double current_area =
      static_cast<double>(current.width) * static_cast<double>(current.height);
    if (std::abs(candidate_area - current_area) > 0.15 * std::max(candidate_area, current_area)) {
      return candidate_area > current_area;
    }
    return candidate.confidence > current.confidence;
  }

  // 연속 검출 횟수와 필터를 적용해 현재 부표 정보를 갱신한다.
  void update_detection_slot(
    std::optional<Detection> & slot, Detection incoming)
  {
    const auto received_at = now();
    int hits = 1;
    if (recent(slot) && same_target(incoming, *slot)) {
      hits = slot->consecutive_hits + 1;
      const double dt = (received_at - slot->received_at).seconds();
      incoming.center_x =
        static_cast<float>(low_pass(slot->center_x, incoming.center_x, dt));
      incoming.center_y =
        static_cast<float>(low_pass(slot->center_y, incoming.center_y, dt));
      incoming.width = static_cast<float>(low_pass(slot->width, incoming.width, dt));
      incoming.height =
        static_cast<float>(low_pass(slot->height, incoming.height, dt));
    }
    incoming.received_at = received_at;
    incoming.consecutive_hits = hits;
    slot = incoming;
  }

  // 두 검출 중심의 차이로 같은 부표인지 판단한다.
  bool same_target(const Detection & a, const Detection & b) const
  {
    const double width = std::max(a.image_width, b.image_width);
    const double height = std::max(a.image_height, b.image_height);
    return
      std::abs(a.center_x - b.center_x) / width <= same_target_center_ratio_ &&
      std::abs(a.center_y - b.center_y) / height <= same_target_center_ratio_;
  }

  // 이전 값과 새 값을 시간 간격에 따라 부드럽게 결합한다.
  double low_pass(const double previous, const double sample, const double dt) const
  {
    if (lpf_tau_sec_ <= 1.0e-9 || dt <= 0.0) {
      return sample;
    }
    const double alpha = dt / (lpf_tau_sec_ + dt);
    return alpha * sample + (1.0 - alpha) * previous;
  }

  // 회전 탐색 중 가장 큰 부표 후보와 관측 방향을 저장한다.
  void consider_scan_candidate(const Detection & detection)
  {
    const double area = detection_area_ratio(detection);
    if (
      !scan_candidate_ || area > scan_candidate_->area_ratio ||
      (std::abs(area - scan_candidate_->area_ratio) <= 1.0e-6 &&
      detection.confidence > scan_candidate_->detection.confidence))
    {
      scan_candidate_ = ScanCandidate{detection, current_yaw_rad_, area};
    }
  }

  // 탐색 요청을 받으면 이전 기록을 초기화하고 부표 확인을 시작한다.
  void on_vision_search_request(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || state_ != State::IDLE) {
      return;
    }
    reset_mission();
    publish_target_confirmed(false);
    transition_to(State::TARGET_CONFIRM, "hydrophone requested visual confirmation");
  }

  // 제어권 승인 시 현재 수심을 고정하고 초기 부표 처리를 시작한다.
  void on_vision_control_granted(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      control_grant_pending_ = false;
      return;
    }
    if (
      state_ != State::IDLE && state_ != State::TARGET_CONFIRM &&
      state_ != State::WAIT_CONTROL_GRANT)
    {
      return;
    }

    control_grant_pending_ = true;
    try_activate_control_grant();
  }

  // A transient-local grant may arrive before start_frame/odom/depth.  Keep RC
  // silent until all inputs share a valid arena frame instead of entering an
  // unrecoverable launch-order FAILSAFE.
  void try_activate_control_grant()
  {
    if (!control_grant_pending_ || vision_has_control_) {
      return;
    }
    if (require_physical_detach_ && !have_physical_detach_count_) {
      return;
    }
    if (
      !required_fcu_mode_.empty() &&
      current_fcu_mode_ != required_fcu_mode_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for FCU mode %s (current=%s)",
        required_fcu_mode_.c_str(),
        current_fcu_mode_.empty() ? "unknown" : current_fcu_mode_.c_str());
      return;
    }
    if (!have_recent_odometry()) {
      return;
    }
    const auto depth = current_depth();
    if (!depth) {
      return;
    }
    control_grant_pending_ = false;
    vision_has_control_ = true;
    mission_hold_depth_m_ = *depth;
    reset_depth_pid();
    handoff_position_ = current_position_;
    target_detach_baseline_count_ = physical_detach_count_;

    if (confirmed_buoy()) {
      begin_target(TargetContext::INITIAL, "confirmed hydrophone handoff target");
    } else {
      begin_initial_scan();
    }
  }

  // 비상정지 요청을 받으면 안전 정지 상태로 전환한다.
  void on_emergency_stop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    transition_to(State::FAILSAFE, "emergency stop");
  }

  void on_physical_detach_count(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    physical_detach_count_ = msg->data;
    have_physical_detach_count_ = true;
  }

  void on_fcu_state(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_fcu_mode_ = msg->mode;
  }

  // 새 임무를 시작할 수 있도록 진행 상태와 제어 누적값을 초기화한다.
  void reset_mission()
  {
    buoy_.reset();
    stick_.reset();
    scan_candidate_.reset();
    active_lane_index_.reset();
    lane_completed_.assign(lane_planner_->lanes().size(), false);
    target_confirm_started_at_.reset();
    detection_confirm_started_at_.reset();
    ignore_detections_until_progress_m_.reset();
    mission_hold_depth_m_.reset();
    target_departure_hold_depth_m_.reset();
    vision_has_control_ = false;
    target_retry_count_ = 0;
    target_lost_started_at_.reset();
    reset_waypoint_pid();
    reset_depth_pid();
  }

  // 입력 안전성을 검사하고 현재 상태에 맞는 제어 동작을 주기적으로 수행한다.
  void on_timer()
  {
    try_activate_control_grant();
    if (state_ == State::IDLE) {
      return;
    }
    if (state_ == State::TARGET_CONFIRM) {
      run_target_confirm();
      return;
    }
    if (state_ == State::WAIT_CONTROL_GRANT || !vision_has_control_) {
      return;
    }

    auto channels = nochange_channels();
    if (state_ == State::COMPLETE || state_ == State::FAILSAFE) {
      release_controlled_channels(channels);
      publish_channels(channels);
      return;
    }

    if (!have_recent_odometry()) {
      transition_to(State::FAILSAFE, "odometry stale");
    } else if (
      !required_fcu_mode_.empty() &&
      current_fcu_mode_ != required_fcu_mode_)
    {
      transition_to(State::FAILSAFE, "FCU left required direct-throttle mode");
    } else if (!current_depth() || !mission_hold_depth_m_) {
      transition_to(State::FAILSAFE, "depth stale");
    } else if (*current_depth() > max_depth_m_) {
      transition_to(State::FAILSAFE, "maximum depth exceeded");
    }
    if (state_ == State::FAILSAFE) {
      release_controlled_channels(channels);
      publish_channels(channels);
      return;
    }

    switch (state_) {
      case State::IDLE:
      case State::TARGET_CONFIRM:
      case State::WAIT_CONTROL_GRANT:
      case State::COMPLETE:
      case State::FAILSAFE:
        break;
      case State::INITIAL_SCAN_360:
        run_initial_scan(channels);
        break;
      case State::TURN_TO_INITIAL_TARGET:
        run_turn_to_initial_target(channels);
        break;
      case State::FIND_AND_ALIGN:
        run_find_and_align(channels);
        break;
      case State::ALIGN_STICK:
        run_align_stick(channels);
        break;
      case State::INSERT_FORK:
        run_insert_fork(channels);
        break;
      case State::DETACH:
        run_detach(channels);
        break;
      case State::GO_BACK:
        run_go_back(channels);
        break;
      case State::VERIFY_RELEASE:
        run_verify_release(channels);
        break;
      case State::MOVE_TO_LANE_START:
        run_move_to_lane_start(channels);
        break;
      case State::LANE_FOLLOWING_WITH_SEARCH:
        run_lane_following(channels);
        break;
      case State::RETURN_TO_ACTIVE_LANE:
        run_return_to_active_lane(channels);
        break;
    }

    if (state_ == State::COMPLETE || state_ == State::FAILSAFE) {
      channels = nochange_channels();
      release_controlled_channels(channels);
    }
    publish_channels(channels);
  }

  // 연속 검출 조건을 만족한 부표를 확정해 제어권 승인을 요청한다.
  void run_target_confirm()
  {
    if (!confirmed_buoy()) {
      target_confirm_started_at_.reset();
      return;
    }
    if (!target_confirm_started_at_) {
      target_confirm_started_at_ = now();
      return;
    }
    if ((now() - *target_confirm_started_at_).seconds() < target_confirm_sec_) {
      return;
    }
    publish_target_confirmed(true);
    transition_to(State::WAIT_CONTROL_GRANT, "stable buoy confirmed");
  }

  // 초기 부표가 없을 때 한 바퀴 회전을 위한 누적값을 초기화한다.
  void begin_initial_scan()
  {
    scan_candidate_.reset();
    scan_accumulated_yaw_rad_ = 0.0;
    scan_previous_yaw_rad_ = current_yaw_rad_;
    transition_to(State::INITIAL_SCAN_360, "no confirmed target; starting 360-degree scan");
  }

  // 제자리에서 한 바퀴 회전하며 가장 가까운 부표 후보를 찾는다.
  void run_initial_scan(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    set_channel(channels, yaw_channel_, initial_scan_yaw_pwm_);

    const double delta = std::abs(wrap_pi(current_yaw_rad_ - scan_previous_yaw_rad_));
    if (delta < 0.5) {
      scan_accumulated_yaw_rad_ += delta;
    }
    scan_previous_yaw_rad_ = current_yaw_rad_;
    if (
      scan_accumulated_yaw_rad_ <
      2.0 * M_PI - initial_scan_completion_tolerance_rad_)
    {
      return;
    }

    if (!scan_candidate_) {
      select_next_lane("360-degree scan completed without a buoy");
      return;
    }
    initial_target_yaw_rad_ = scan_candidate_->arena_yaw_rad;
    buoy_.reset();
    stick_.reset();
    reset_waypoint_pid();
    transition_to(State::TURN_TO_INITIAL_TARGET, "nearest scan candidate selected");
  }

  // 회전 탐색에서 선택한 방향으로 돌며 초기 부표를 다시 찾는다.
  void run_turn_to_initial_target(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    set_heading_control(channels, initial_target_yaw_rad_);
    if (recent(buoy_)) {
      begin_target(TargetContext::INITIAL, "initial scan target reacquired");
      return;
    }
    if (state_age_sec() >= initial_target_reacquire_timeout_sec_) {
      select_next_lane("initial scan target could not be reacquired");
    }
  }

  // 부표의 발견 위치와 처리 구간을 저장하고 접근 상태로 전환한다.
  void begin_target(const TargetContext context, const std::string & reason)
  {
    target_context_ = context;
    target_retry_count_ = 0;
    target_lost_started_at_.reset();
    target_detach_baseline_count_ = physical_detach_count_;
    if (context == TargetContext::LANE) {
      if (!active_lane_index_) {
        transition_to(State::FAILSAFE, "lane target selected without an active lane");
        return;
      }
      target_departure_progress_m_ = active_lane_progress();
      target_departure_hold_depth_m_ = mission_hold_depth_m_;
    } else {
      target_departure_hold_depth_m_.reset();
    }
    publish_lane_event(
      context == TargetContext::INITIAL ? "target_initial" : "target_lane");
    transition_to(State::FIND_AND_ALIGN, reason);
  }

  // 부표를 화면 중앙에 정렬하고 class-1 stick이 안정적으로 보이는 거리까지 접근한다.
  void run_find_and_align(std::array<uint16_t, 18> & channels)
  {
    if (!recent(buoy_)) {
      set_neutral_control(channels);
      hold_mission_depth(channels);
      if (!target_lost_started_at_) {
        target_lost_started_at_ = now();
      }
      if (
        buoy_ &&
        (now() - *target_lost_started_at_).seconds() < target_reacquire_timeout_sec_)
      {
        apply_visual_tracking(
          channels, *buoy_, align_target_x_, align_target_y_, neutral_pwm_,
          align_deadband_x_, align_deadband_y_);
        if (target_context_ == TargetContext::LANE) {
          set_channel(channels, throttle_channel_, mission_depth_pwm());
        }
        return;
      }
      finish_target(true, "buoy could not be reacquired during approach");
      return;
    }
    target_lost_started_at_.reset();

    const auto [error_x, error_y] =
      normalized_error(*buoy_, align_target_x_, align_target_y_);
    const bool vertical_aligned =
      target_context_ == TargetContext::LANE ||
      std::abs(error_y) <= align_deadband_y_;
    const bool aligned =
      std::abs(error_x) <= align_deadband_x_ &&
      vertical_aligned;
    if (target_excursion_limit_reached()) {
      finish_target(true, "target incapable within allowed excursion");
      return;
    }
    if (detection_area_ratio(*buoy_) >= approach_area_ratio_ && recent(stick_)) {
      transition_to(State::ALIGN_STICK, "close buoy and associated stick visible");
      set_neutral_control(channels);
      hold_mission_depth(channels);
      return;
    }
    apply_visual_tracking(
      channels, *buoy_, align_target_x_, align_target_y_,
      aligned ? approach_forward_pwm_from_area(*buoy_) : neutral_pwm_,
      align_deadband_x_, align_deadband_y_);
    if (target_context_ == TargetContext::LANE) {
      // Every competition underwater float has the same attachment depth.
      // Chasing a noisy/partial buoy bbox vertically in STABILIZE applies the
      // +/-110 PWM dead-zone step continuously and can climb several metres.
      // Preserve the coverage depth and use vision only for yaw/forward.
      set_channel(channels, throttle_channel_, mission_depth_pwm());
    }
  }

  int approach_forward_pwm_from_area(const Detection & buoy) const
  {
    const double ratio = detection_area_ratio(buoy);
    const double remaining = std::max(0.0, approach_area_ratio_ - ratio);
    const int floor_pwm = std::min(approach_forward_min_pwm_, approach_forward_pwm_);
    const int span = std::max(0, approach_forward_pwm_ - floor_pwm);
    const double gain =
      approach_area_ratio_ > 1.0e-9 ?
      static_cast<double>(span) / approach_area_ratio_ : 0.0;
    return std::clamp(
      floor_pwm + static_cast<int>(std::lround(gain * remaining)),
      floor_pwm, approach_forward_pwm_);
  }

  void run_align_stick(std::array<uint16_t, 18> & channels)
  {
    if (!recent(stick_)) {
      set_neutral_control(channels);
      hold_mission_depth(channels);
      transition_to(
        State::FIND_AND_ALIGN,
        "associated stick lost; returning to buoy approach");
      return;
    }

    const double target_y = active_fork_target_y();
    apply_visual_tracking(
      channels, *stick_, fork_target_x_, target_y, neutral_pwm_,
      stick_deadband_x_, stick_deadband_y_);
    if (target_context_ == TargetContext::LANE) {
      set_channel(channels, throttle_channel_, mission_depth_pwm());
    }
    const auto [error_x, error_y] =
      normalized_error(*stick_, fork_target_x_, target_y);
    const bool vertical_aligned =
      target_context_ == TargetContext::LANE ||
      std::abs(error_y) <= stick_deadband_y_;
    if (
      std::abs(error_x) <= stick_deadband_x_ &&
      vertical_aligned)
    {
      if (!alignment_started_at_) {
        alignment_started_at_ = now();
      } else if ((now() - *alignment_started_at_).seconds() >= align_stable_sec_) {
        const auto aligned_depth = current_depth();
        if (!aligned_depth) {
          transition_to(State::FAILSAFE, "depth unavailable at stick alignment");
          return;
        }
        // INSERT/DETACH use depth hold rather than visual vertical control.
        // Latch the physically aligned depth here; keeping the handoff depth
        // would immediately move the rake back above/below the PVC target.
        if (
          target_context_ == TargetContext::LANE &&
          target_departure_hold_depth_m_)
        {
          mission_hold_depth_m_ = target_departure_hold_depth_m_;
        } else {
          const double depth_offset =
            target_context_ == TargetContext::LANE ?
            lane_fork_depth_offset_m_ : 0.0;
          mission_hold_depth_m_ = *aligned_depth + depth_offset;
        }
        reset_depth_pid();
        target_detach_baseline_count_ = physical_detach_count_;
        RCLCPP_INFO(
          get_logger(),
          "Fork alignment latched: stick=(%.3f, %.3f) target=(%.3f, %.3f) "
          "depth=%.3f held_depth=%.3f world=(%.3f, %.3f)",
          stick_->center_x / stick_->image_width,
          stick_->center_y / stick_->image_height,
          fork_target_x_, target_y, *aligned_depth, *mission_hold_depth_m_,
          current_position_.x, current_position_.y);
        transition_to(State::INSERT_FORK, "stick alignment stable");
      }
    } else {
      alignment_started_at_.reset();
    }
  }

  // 초기 반경 또는 활성 레인 오프셋 한계에 도달했는지 확인한다.
  bool target_excursion_limit_reached() const
  {
    if (!lane_planner_->inside_safe_bounds(current_position_)) {
      return true;
    }
    if (target_context_ == TargetContext::INITIAL) {
      return distance(current_position_, handoff_position_) >= initial_search_radius_m_;
    }
    if (!active_lane_index_) {
      return true;
    }
    return lane_planner_->cross_track_distance(
      current_position_, *active_lane_index_) >= lane_search_offset_m_;
  }

  // 설정된 시간 동안 일반 삽입 출력을 적용한다.
  void run_insert_fork(std::array<uint16_t, 18> & channels)
  {
    if (current_target_physically_detached()) {
      transition_to(State::GO_BACK, "physical target detached during fork insertion");
      set_neutral_control(channels);
      hold_mission_depth(channels);
      set_channel(channels, forward_channel_, go_back_pwm_);
      return;
    }
    if (state_age_sec() >= insert_fork_duration_sec_) {
      transition_to(State::DETACH, "fork insertion pulse complete");
      apply_fork_insertion_control(channels, detach_pwm_);
      return;
    }
    apply_fork_insertion_control(channels, insert_fork_pwm_);
  }

  void run_detach(std::array<uint16_t, 18> & channels)
  {
    if (current_target_physically_detached()) {
      transition_to(State::GO_BACK, "physical target detached");
      set_neutral_control(channels);
      hold_mission_depth(channels);
      set_channel(channels, forward_channel_, go_back_pwm_);
      return;
    }
    if (state_age_sec() >= detach_duration_sec_) {
      transition_to(State::GO_BACK, "detach pulse complete");
      set_neutral_control(channels);
      hold_mission_depth(channels);
      set_channel(channels, forward_channel_, go_back_pwm_);
      return;
    }
    apply_fork_insertion_control(channels, detach_pwm_);
  }

  // Keep correcting the stick all the way to contact.  A fixed yaw/depth
  // after far-field alignment turns a small image offset into a large rake
  // miss as the vehicle closes the remaining distance.
  void apply_fork_insertion_control(
    std::array<uint16_t, 18> & channels, const int forward_pwm)
  {
    if (recent(stick_)) {
      apply_visual_tracking(
        channels, *stick_, fork_target_x_, active_fork_target_y(), forward_pwm,
        stick_deadband_x_, stick_deadband_y_);
      if (target_context_ == TargetContext::LANE) {
        // The ordinary class-1 YOLO box has a repeatable vertical centre
        // bias.  Once corrected at alignment, preserve that calibrated depth
        // while continuing only the horizontal visual correction.
        set_channel(channels, throttle_channel_, mission_depth_pwm());
      }
      return;
    }
    set_neutral_control(channels);
    hold_mission_depth(channels);
    set_channel(channels, forward_channel_, forward_pwm);
  }

  double active_fork_target_y() const
  {
    return
      target_context_ == TargetContext::LANE ?
      lane_fork_target_y_ : fork_target_y_;
  }

  // 설정된 시간 동안 후퇴한 뒤 결과 확인으로 전환한다.
  void run_go_back(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    if (state_age_sec() < go_back_duration_sec_) {
      set_channel(channels, forward_channel_, go_back_pwm_);
      return;
    }
    transition_to(State::VERIFY_RELEASE, "backoff complete");
  }

  bool current_target_physically_detached() const
  {
    return
      have_physical_detach_count_ &&
      physical_detach_count_ > target_detach_baseline_count_;
  }

  // 물리 count 증가를 우선 사용하고, 명시적으로 허용한 경우에만 bbox 소실로 대체한다.
  void run_verify_release(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    const bool physical_success = current_target_physically_detached();
    const bool visual_success =
      !require_physical_detach_ && !recent(buoy_) &&
      state_age_sec() >= verify_clear_sec_;
    if (physical_success || visual_success) {
      publish_lane_event(
        physical_success ? "target_detached_physical" : "target_detached_visual");
      finish_target(
        false,
        physical_success ? "physical target detached" : "buoy absent after backoff",
        true);
      return;
    }
    if (state_age_sec() >= verify_timeout_sec_) {
      if (target_retry_count_ < max_target_retries_) {
        ++target_retry_count_;
        alignment_started_at_.reset();
        transition_to(
          recent(stick_) ? State::ALIGN_STICK : State::FIND_AND_ALIGN,
          "physical release not confirmed; retrying target");
      } else {
        publish_lane_event("target_abandoned_retry_limit");
        finish_target(true, "physical release retry limit reached");
      }
    }
  }

  // 부표 처리 기록을 정리하고 시작점 이동 또는 활성 레인 복귀를 준비한다.
  void finish_target(
    const bool skip_repeated_detection, const std::string & reason,
    const bool detached = false)
  {
    buoy_.reset();
    stick_.reset();
    alignment_started_at_.reset();
    target_lost_started_at_.reset();
    detection_confirm_started_at_.reset();
    if (target_context_ == TargetContext::INITIAL) {
      if (require_physical_detach_ && !detached) {
        transition_to(State::FAILSAFE, "initial target was not physically detached");
        return;
      }
      select_next_lane(reason);
      return;
    }
    if (!active_lane_index_) {
      transition_to(State::FAILSAFE, "cannot return because active lane is missing");
      return;
    }
    // After either a detach or an abandoned target, suppress the immediate
    // long-range view from the same departure point.  The detached float
    // rises to the surface, but low-confidence remnants/background boxes can
    // otherwise trigger another multi-minute approach before the lane moves.
    const double lane_length = lane_planner_->lane_length(*active_lane_index_);
    ignore_detections_until_progress_m_ = std::min(
      lane_length, target_departure_progress_m_ + incapable_skip_distance_m_);
    if (target_departure_hold_depth_m_) {
      mission_hold_depth_m_ = target_departure_hold_depth_m_;
      reset_depth_pid();
    }
    target_departure_hold_depth_m_.reset();

    if (skip_repeated_detection) {
      return_waypoint_ =
        lane_planner_->project_to_lane(current_position_, *active_lane_index_);
    } else {
      // Resume where coverage was interrupted.  Projecting the post-detach
      // vehicle position can jump several metres forward along a lane and
      // permanently skip another buoy that was beside the original target.
      const Vec2 lane_delta = active_lane_finish_ - active_lane_start_;
      const double lane_length = norm(lane_delta);
      const double progress = std::clamp(
        target_departure_progress_m_, 0.0, lane_length);
      return_waypoint_ =
        lane_length > 1.0e-9 ?
        active_lane_start_ + lane_delta * (progress / lane_length) :
        active_lane_start_;
    }
    reset_waypoint_pid();
    transition_to(State::RETURN_TO_ACTIVE_LANE, reason);
  }

  // 현재 위치에서 가장 가까운 미완료 레인 끝점을 다음 시작점으로 선택한다.
  void select_next_lane(const std::string & reason)
  {
    const auto choice =
      lane_planner_->closest_uncompleted_endpoint(current_position_, lane_completed_);
    if (!choice) {
      active_lane_index_.reset();
      publish_lane_event("coverage_complete");
      publish_success(true);
      transition_to(State::COMPLETE, "all generated lanes completed");
      return;
    }

    active_lane_index_ = choice->lane_index;
    active_lane_start_at_a_ = choice->start_at_a;
    active_lane_start_ = choice->start;
    active_lane_finish_ = choice->finish;
    waypoint_ = active_lane_start_;
    ignore_detections_until_progress_m_.reset();
    reset_waypoint_pid();
    transition_to(State::MOVE_TO_LANE_START, reason);
    publish_lane_event("lane_selected:" + std::to_string(*active_lane_index_));
    RCLCPP_INFO(
      get_logger(),
      "Selected lane %zu start=(%.2f, %.2f) finish=(%.2f, %.2f)",
      *active_lane_index_, active_lane_start_.x, active_lane_start_.y,
      active_lane_finish_.x, active_lane_finish_.y);
  }

  // 선택된 레인 끝점으로 이동한 뒤 레인 주행을 시작한다.
  void run_move_to_lane_start(std::array<uint16_t, 18> & channels)
  {
    if (follow_waypoint(channels, waypoint_, lane_forward_pwm_)) {
      reset_waypoint_pid();
      detection_confirm_started_at_.reset();
      transition_to(State::LANE_FOLLOWING_WITH_SEARCH, "active lane start reached");
    }
  }

  // 활성 레인의 끝점을 향해 주행하면서 부표를 계속 확인한다.
  void run_lane_following(std::array<uint16_t, 18> & channels)
  {
    if (!active_lane_index_) {
      transition_to(State::FAILSAFE, "lane following without an active lane");
      return;
    }
    const Vec2 lane_delta = active_lane_finish_ - active_lane_start_;
    const double lane_length = norm(lane_delta);
    const double along_track =
      lane_length > 1.0e-9 ?
      dot(current_position_ - active_lane_start_, lane_delta) / lane_length : 0.0;
    const bool endpoint_reached =
      distance(current_position_, active_lane_finish_) <= waypoint_reach_tolerance_m_;
    const bool endpoint_passed =
      along_track >= lane_length - waypoint_reach_tolerance_m_ &&
      lane_planner_->cross_track_distance(
      current_position_, *active_lane_index_) <=
      std::max(0.5, 2.0 * waypoint_reach_tolerance_m_);
    if (endpoint_reached || endpoint_passed) {
      const auto completed_index = *active_lane_index_;
      lane_completed_[completed_index] = true;
      const auto completed_count = static_cast<std::size_t>(
        std::count(lane_completed_.begin(), lane_completed_.end(), true));
      publish_lane_event(
        "lane_completed:" + std::to_string(completed_index) +
        ":count=" + std::to_string(completed_count));
      active_lane_index_.reset();
      select_next_lane("active lane completed");
      return;
    }

    follow_waypoint(channels, active_lane_finish_, lane_forward_pwm_);

    const double progress = active_lane_progress();
    if (
      ignore_detections_until_progress_m_ &&
      progress + waypoint_reach_tolerance_m_ >= *ignore_detections_until_progress_m_)
    {
      ignore_detections_until_progress_m_.reset();
      buoy_.reset();
    }
    if (ignore_detections_until_progress_m_) {
      detection_confirm_started_at_.reset();
      return;
    }

    if (!confirmed_buoy()) {
      detection_confirm_started_at_.reset();
      return;
    }
    if (!detection_confirm_started_at_) {
      detection_confirm_started_at_ = now();
      return;
    }
    if ((now() - *detection_confirm_started_at_).seconds() >= target_confirm_sec_) {
      begin_target(TargetContext::LANE, "stable buoy detected during lane coverage");
      set_neutral_control(channels);
      hold_mission_depth(channels);
    }
  }

  // 부표 작업 후 활성 레인의 가장 가까운 지점으로 복귀한다.
  void run_return_to_active_lane(std::array<uint16_t, 18> & channels)
  {
    if (!active_lane_index_) {
      transition_to(State::FAILSAFE, "return requested without an active lane");
      return;
    }
    if (follow_waypoint(channels, return_waypoint_, lane_forward_pwm_)) {
      reset_waypoint_pid();
      buoy_.reset();
      detection_confirm_started_at_.reset();
      transition_to(
        State::LANE_FOLLOWING_WITH_SEARCH,
        "nearest point on the active lane reached");
    }
  }

  // 목표 위치를 향해 방향을 맞추고 허용 오차 안에서 전진한다.
  bool follow_waypoint(
    std::array<uint16_t, 18> & channels, const Vec2 & target, const int forward_pwm)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    const Vec2 delta = target - current_position_;
    if (norm(delta) <= waypoint_reach_tolerance_m_) {
      reset_waypoint_pid();
      return true;
    }

    const double desired_yaw = std::atan2(delta.y, delta.x);
    const double yaw_error = wrap_pi(desired_yaw - current_yaw_rad_);
    set_heading_control(channels, desired_yaw);
    if (std::abs(yaw_error) <= waypoint_heading_tolerance_rad_) {
      set_channel(channels, forward_channel_, forward_pwm);
    }
    return false;
  }

  // 목표 방향과 현재 방향의 오차로 회전 채널 출력을 계산한다.
  void set_heading_control(
    std::array<uint16_t, 18> & channels, const double desired_yaw)
  {
    const auto current_time = now();
    const double error = wrap_pi(desired_yaw - current_yaw_rad_);
    double dt = 0.0;
    double derivative = 0.0;
    if (waypoint_pid_initialized_) {
      dt = std::clamp(
        (current_time - waypoint_pid_time_).seconds(), 0.0, 0.2);
      if (dt > 1.0e-6) {
        derivative = (error - waypoint_previous_error_) / dt;
      }
    }
    waypoint_integral_ = std::clamp(
      waypoint_integral_ + error * dt,
      -waypoint_yaw_integral_limit_, waypoint_yaw_integral_limit_);
    waypoint_pid_time_ = current_time;
    waypoint_previous_error_ = error;
    waypoint_pid_initialized_ = true;

    double command =
      waypoint_yaw_kp_ * error +
      waypoint_yaw_ki_ * waypoint_integral_ +
      waypoint_yaw_kd_ * derivative;
    if (waypoint_yaw_invert_) {
      command = -command;
    }
    int delta_pwm = std::clamp(
      static_cast<int>(std::lround(command * rc_pwm_span_)),
      -max_waypoint_yaw_delta_pwm_, max_waypoint_yaw_delta_pwm_);
    if (
      std::abs(error) > waypoint_heading_tolerance_rad_ &&
      min_effective_yaw_delta_pwm_ > 0)
    {
      const double signed_error = waypoint_yaw_invert_ ? -error : error;
      delta_pwm = effective_tracking_delta(
        delta_pwm, signed_error, 0.0, min_effective_yaw_delta_pwm_);
    }
    set_channel(channels, yaw_channel_, neutral_pwm_ + delta_pwm);
  }

  // 화면 오차와 수심 오차를 결합해 부표 추적 출력을 계산한다.
  void apply_visual_tracking(
    std::array<uint16_t, 18> & channels, const Detection & detection,
    const double target_x, const double target_y, const int forward_pwm,
    const double horizontal_deadband, const double vertical_deadband)
  {
    set_neutral_control(channels);
    const auto [error_x, error_y] =
      normalized_error(detection, target_x, target_y);

    const double yaw_sign = vision_yaw_invert_ ? -1.0 : 1.0;
    const double signed_yaw_error = yaw_sign * error_x;
    int yaw_delta = static_cast<int>(
      signed_yaw_error * static_cast<double>(max_vision_yaw_delta_pwm_));
    yaw_delta = effective_tracking_delta(
      yaw_delta, signed_yaw_error, horizontal_deadband,
      min_effective_yaw_delta_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_ + yaw_delta);

    const double vertical_sign = vertical_positive_is_up_ ? -1.0 : 1.0;
    const double signed_vertical_error = vertical_sign * error_y;
    int vision_delta = static_cast<int>(
      signed_vertical_error * static_cast<double>(max_vision_throttle_delta_pwm_));
    vision_delta = effective_tracking_delta(
      vision_delta, signed_vertical_error, vertical_deadband,
      min_effective_vertical_delta_pwm_);
    const int vision_throttle = neutral_pwm_ + vision_delta;
    const int depth_throttle = mission_depth_pwm();
    const double weight = std::clamp(
      std::max(
        approach_vision_throttle_weight_,
        std::abs(error_y) / vertical_full_weight_error_),
      0.0, 1.0);
    int blended_throttle = static_cast<int>(std::lround(
      weight * static_cast<double>(vision_throttle) +
      (1.0 - weight) * static_cast<double>(depth_throttle)));
    if (std::abs(error_y) > vertical_deadband) {
      int blended_delta = blended_throttle - neutral_pwm_;
      blended_delta = effective_tracking_delta(
        blended_delta, signed_vertical_error, vertical_deadband,
        min_effective_vertical_delta_pwm_);
      blended_throttle = neutral_pwm_ + blended_delta;
    }
    set_channel(channels, throttle_channel_, blended_throttle);
    set_channel(channels, forward_channel_, forward_pwm);
  }

  static int effective_tracking_delta(
    const int requested_delta, const double signed_error,
    const double normalized_deadband, const int minimum_magnitude)
  {
    if (minimum_magnitude <= 0) {
      return requested_delta;
    }
    if (std::abs(signed_error) <= normalized_deadband) {
      return 0;
    }
    const int magnitude =
      std::max(std::abs(requested_delta), minimum_magnitude);
    return signed_error < 0.0 ? -magnitude : magnitude;
  }

  // 검출 중심과 목표점의 차이를 정규화된 화면 오차로 변환한다.
  std::pair<double, double> normalized_error(
    const Detection & detection, const double target_x, const double target_y) const
  {
    const double x = detection.center_x / detection.image_width;
    const double y = detection.center_y / detection.image_height;
    return {
      std::clamp(2.0 * (x - target_x), -1.0, 1.0),
      std::clamp(2.0 * (y - target_y), -1.0, 1.0)};
  }

  // 부표 상자가 왼쪽 화면 절반에서 차지하는 면적 비율을 계산한다.
  double left_half_fill_ratio(const Detection & detection) const
  {
    const double image_width = detection.image_width;
    const double image_height = detection.image_height;
    const double box_x_min = detection.center_x - 0.5 * detection.width;
    const double box_x_max = detection.center_x + 0.5 * detection.width;
    const double box_y_min = detection.center_y - 0.5 * detection.height;
    const double box_y_max = detection.center_y + 0.5 * detection.height;
    const double intersection_width =
      std::max(0.0, std::min(box_x_max, 0.5 * image_width) - std::max(box_x_min, 0.0));
    const double intersection_height =
      std::max(0.0, std::min(box_y_max, image_height) - std::max(box_y_min, 0.0));
    return (intersection_width * intersection_height) /
           (0.5 * image_width * image_height);
  }

  // 부표 상자가 전체 화면에서 차지하는 면적 비율을 계산한다.
  double detection_area_ratio(const Detection & detection) const
  {
    return static_cast<double>(detection.width * detection.height) /
           static_cast<double>(detection.image_width * detection.image_height);
  }

  // 주 수심 입력을 우선하고 유효하지 않으면 보조 입력을 반환한다.
  std::optional<double> current_depth() const
  {
    const auto current_time = now();
    if (
      pose_depth_ &&
      (current_time - pose_depth_->received_at).seconds() <= depth_timeout_sec_)
    {
      return pose_depth_->depth_m;
    }
    if (
      scalar_depth_ &&
      (current_time - scalar_depth_->received_at).seconds() <= depth_timeout_sec_)
    {
      return scalar_depth_->depth_m;
    }
    return std::nullopt;
  }

  // 저장된 목표수심과 현재수심의 차이로 수직 채널 출력을 계산한다.
  int mission_depth_pwm()
  {
    const auto depth = current_depth();
    if (!depth || !mission_hold_depth_m_) {
      return neutral_pwm_;
    }

    const auto current_time = now();
    const double error = *mission_hold_depth_m_ - *depth;
    double dt = 0.0;
    double derivative = 0.0;
    if (depth_pid_initialized_) {
      dt = std::clamp((current_time - depth_pid_time_).seconds(), 0.0, 0.2);
      if (dt > 1.0e-6) {
        derivative = (error - depth_previous_error_) / dt;
      }
    }
    depth_integral_ = std::clamp(
      depth_integral_ + error * dt,
      -depth_integral_limit_m_sec_, depth_integral_limit_m_sec_);
    depth_pid_time_ = current_time;
    depth_previous_error_ = error;
    depth_pid_initialized_ = true;

    int delta = static_cast<int>(std::lround(
      depth_kp_pwm_per_m_ * error +
      depth_ki_pwm_per_m_sec_ * depth_integral_ +
      depth_kd_pwm_sec_per_m_ * derivative));
    delta += buoyancy_hold_delta_pwm_;
    delta = std::clamp(delta, -max_depth_delta_pwm_, max_depth_delta_pwm_);
    const int direction = vertical_positive_is_up_ ? -1 : 1;
    int output_delta = direction * delta;
    if (std::abs(error) > depth_deadband_m_) {
      output_delta = effective_tracking_delta(
        output_delta, static_cast<double>(direction) * error,
        depth_deadband_m_, min_effective_vertical_delta_pwm_);
    }
    return neutral_pwm_ + output_delta;
  }

  // 현재 채널 배열에 목표수심 유지 출력을 적용한다.
  void hold_mission_depth(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, mission_depth_pwm());
  }

  // 활성 레인의 시작점 기준 현재 진행 거리를 반환한다.
  double active_lane_progress() const
  {
    if (!active_lane_index_) {
      return 0.0;
    }
    return lane_planner_->progress_from_start(
      current_position_, *active_lane_index_, active_lane_start_at_a_);
  }

  // 검출 정보가 존재하고 허용 시간 안에 수신되었는지 확인한다.
  bool recent(const std::optional<Detection> & detection) const
  {
    return detection &&
           (now() - detection->received_at).seconds() <= detection_timeout_sec_;
  }

  // 최근 부표의 연속 검출 횟수가 확정 기준을 만족하는지 확인한다.
  bool confirmed_buoy() const
  {
    return recent(buoy_) && buoy_->consecutive_hits >= target_confirm_hits_;
  }

  // 위치 정보가 존재하고 허용 시간 안에 수신되었는지 확인한다.
  bool have_recent_odometry() const
  {
    return have_odometry_ &&
           (now() - odometry_received_at_).seconds() <= odometry_timeout_sec_;
  }

  // 현재 상태에 진입한 뒤 지난 시간을 초 단위로 반환한다.
  double state_age_sec() const
  {
    return (now() - state_entered_at_).seconds();
  }

  // 상태와 진입 시각을 갱신하고 변경 내용을 발행한다.
  void transition_to(const State next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "State %s -> %s: %s",
      state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    alignment_started_at_.reset();
    publish_state();
  }

  // 내부 상태 값을 외부에 표시할 문자열로 변환한다.
  static const char * state_name(const State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::TARGET_CONFIRM: return "TARGET_CONFIRM";
      case State::WAIT_CONTROL_GRANT: return "WAIT_CONTROL_GRANT";
      case State::INITIAL_SCAN_360: return "INITIAL_SCAN_360";
      case State::TURN_TO_INITIAL_TARGET: return "TURN_TO_INITIAL_TARGET";
      case State::FIND_AND_ALIGN: return "FIND_AND_ALIGN";
      case State::ALIGN_STICK: return "ALIGN_STICK";
      case State::INSERT_FORK: return "INSERT_FORK";
      case State::DETACH: return "DETACH";
      case State::GO_BACK: return "GO_BACK";
      case State::VERIFY_RELEASE: return "VERIFY_RELEASE";
      case State::MOVE_TO_LANE_START: return "MOVE_TO_LANE_START";
      case State::LANE_FOLLOWING_WITH_SEARCH: return "LANE_FOLLOWING_WITH_SEARCH";
      case State::RETURN_TO_ACTIVE_LANE: return "RETURN_TO_ACTIVE_LANE";
      case State::COMPLETE: return "COMPLETE";
      case State::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  // 현재 상태 문자열을 상태 토픽으로 발행한다.
  void publish_state()
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    state_pub_->publish(msg);
  }

  // 부표 확정 여부를 제어권 인계용 토픽으로 발행한다.
  void publish_target_confirmed(const bool confirmed)
  {
    if (!target_confirmed_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = confirmed;
    target_confirmed_pub_->publish(msg);
  }

  void publish_lane_event(const std::string & event)
  {
    if (!lane_event_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = event;
    lane_event_pub_->publish(msg);
  }

  void publish_success(const bool success)
  {
    if (!success_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = success;
    success_pub_->publish(msg);
  }

  // 모든 채널을 변경하지 않음 값으로 채운 배열을 만든다.
  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  // 제어하는 세 채널을 중립 출력으로 설정한다.
  void set_neutral_control(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, throttle_channel_, neutral_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_);
    set_channel(channels, forward_channel_, neutral_pwm_);
  }

  // 제어하는 세 채널의 강제 출력을 해제한다.
  void release_controlled_channels(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, throttle_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, yaw_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, forward_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  // 채널 번호를 배열 위치로 바꾸고 출력 범위를 제한해 저장한다.
  void set_channel(
    std::array<uint16_t, 18> & channels, const int channel, int pwm) const
  {
    if (
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE &&
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE)
    {
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<std::size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  // 같은 채널 명령을 실제 제어 토픽과 확인 토픽에 발행한다.
  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels = channels;
    rc_pub_->publish(msg);
    rc_monitor_pub_->publish(msg);
  }

  // 위치 이동에 사용하는 방향 제어 누적값을 초기화한다.
  void reset_waypoint_pid()
  {
    waypoint_pid_initialized_ = false;
    waypoint_integral_ = 0.0;
    waypoint_previous_error_ = 0.0;
  }

  // 수심 유지에 사용하는 제어 누적값을 초기화한다.
  void reset_depth_pid()
  {
    depth_pid_initialized_ = false;
    depth_integral_ = 0.0;
    depth_previous_error_ = 0.0;
  }

  std::string bbox_topic_;
  std::string depth_topic_;
  std::string depth_pose_topic_;
  std::string odometry_topic_;
  std::string start_frame_topic_;
  std::string vision_search_request_topic_;
  std::string target_confirmed_topic_;
  std::string vision_control_granted_topic_;
  std::string emergency_stop_topic_;
  std::string state_topic_;
  std::string rc_override_topic_;
  std::string rc_monitor_topic_;
  std::string fcu_state_topic_;
  std::string physical_detach_count_topic_;
  std::string lane_event_topic_;
  std::string success_topic_;
  std::string required_fcu_mode_;

  double arena_length_m_{15.0};
  double arena_width_m_{16.0};
  double arena_offset_x_m_{-0.3};
  double arena_offset_y_m_{0.3};
  double arena_safety_margin_m_{0.5};
  std::string arena_start_corner_{"bottom_left"};
  double lane_search_offset_m_{2.0};
  double incapable_skip_distance_m_{2.0};
  double waypoint_reach_tolerance_m_{0.15};
  int expected_lane_count_{0};

  double control_rate_hz_{20.0};
  double odometry_timeout_sec_{0.5};
  double detection_timeout_sec_{0.7};
  double depth_timeout_sec_{3.0};
  double depth_pose_scale_{-1.0};
  double depth_pose_offset_m_{0.0};
  double max_depth_m_{10.5};
  int buoy_class_id_{0};
  int stick_class_id_{1};
  int target_confirm_hits_{4};
  double target_confirm_sec_{0.3};
  double same_target_center_ratio_{0.12};
  double lpf_tau_sec_{0.3};
  double initial_search_radius_m_{2.0};
  int initial_scan_yaw_pwm_{1600};
  double initial_scan_completion_tolerance_rad_{0.15};
  double initial_target_reacquire_timeout_sec_{5.0};
  double target_reacquire_timeout_sec_{2.0};
  double align_target_x_{0.50};
  double align_target_y_{0.50};
  double align_deadband_x_{0.08};
  double align_deadband_y_{0.10};
  double approach_area_ratio_{0.0035};
  int approach_forward_min_pwm_{1540};
  double fork_target_x_{0.44};
  double fork_target_y_{0.54};
  double lane_fork_target_y_{0.54};
  double lane_fork_depth_offset_m_{0.0};
  double stick_deadband_x_{0.04};
  double stick_deadband_y_{0.08};
  double align_stable_sec_{0.3};
  int insert_fork_pwm_{1700};
  double insert_fork_duration_sec_{2.0};
  int detach_pwm_{1700};
  double detach_duration_sec_{1.0};
  int go_back_pwm_{1350};
  double go_back_duration_sec_{1.0};
  double verify_clear_sec_{1.0};
  double verify_timeout_sec_{3.0};
  int max_target_retries_{2};
  bool require_physical_detach_{false};

  int throttle_channel_{3};
  int yaw_channel_{4};
  int forward_channel_{5};
  int neutral_pwm_{1500};
  int min_pwm_{1300};
  int max_pwm_{1700};
  double rc_pwm_span_{400.0};
  int lane_forward_pwm_{1700};
  int approach_forward_pwm_{1560};
  double waypoint_heading_tolerance_rad_{0.1745};
  double waypoint_yaw_kp_{1.15};
  double waypoint_yaw_ki_{0.15};
  double waypoint_yaw_kd_{0.08};
  double waypoint_yaw_integral_limit_{2.0};
  int max_waypoint_yaw_delta_pwm_{180};
  bool waypoint_yaw_invert_{true};
  int max_vision_yaw_delta_pwm_{180};
  int max_vision_throttle_delta_pwm_{250};
  int min_effective_yaw_delta_pwm_{0};
  int min_effective_vertical_delta_pwm_{0};
  bool vision_yaw_invert_{false};
  bool vertical_positive_is_up_{true};
  double approach_vision_throttle_weight_{0.4};
  double vertical_full_weight_error_{0.25};
  double depth_kp_pwm_per_m_{120.0};
  double depth_ki_pwm_per_m_sec_{20.0};
  double depth_kd_pwm_sec_per_m_{16.0};
  double depth_integral_limit_m_sec_{2.0};
  int buoyancy_hold_delta_pwm_{40};
  int max_depth_delta_pwm_{160};
  double depth_deadband_m_{0.04};

  ArenaFrameTransform arena_transform_;
  std::unique_ptr<LanePlanner> lane_planner_;
  State state_{State::IDLE};
  TargetContext target_context_{TargetContext::INITIAL};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  bool vision_has_control_{false};
  bool control_grant_pending_{false};
  std::string current_fcu_mode_;
  bool have_odometry_{false};
  Vec2 current_position_{};
  double current_yaw_rad_{0.0};
  rclcpp::Time odometry_received_at_{0, 0, RCL_ROS_TIME};
  std::optional<DepthSample> pose_depth_;
  std::optional<DepthSample> scalar_depth_;
  std::optional<double> mission_hold_depth_m_;
  std::optional<double> target_departure_hold_depth_m_;
  std::optional<Detection> buoy_;
  std::optional<Detection> stick_;
  std::optional<rclcpp::Time> target_confirm_started_at_;
  std::optional<rclcpp::Time> detection_confirm_started_at_;

  Vec2 handoff_position_{};
  std::optional<ScanCandidate> scan_candidate_;
  double scan_previous_yaw_rad_{0.0};
  double scan_accumulated_yaw_rad_{0.0};
  double initial_target_yaw_rad_{0.0};
  int target_retry_count_{0};
  std::optional<rclcpp::Time> target_lost_started_at_;
  std::optional<rclcpp::Time> alignment_started_at_;
  bool have_physical_detach_count_{false};
  uint32_t physical_detach_count_{0};
  uint32_t target_detach_baseline_count_{0};

  std::vector<bool> lane_completed_;
  std::optional<std::size_t> active_lane_index_;
  bool active_lane_start_at_a_{true};
  Vec2 active_lane_start_{};
  Vec2 active_lane_finish_{};
  Vec2 waypoint_{};
  Vec2 return_waypoint_{};
  double target_departure_progress_m_{0.0};
  std::optional<double> ignore_detections_until_progress_m_;

  bool waypoint_pid_initialized_{false};
  double waypoint_integral_{0.0};
  double waypoint_previous_error_{0.0};
  rclcpp::Time waypoint_pid_time_{0, 0, RCL_ROS_TIME};
  bool depth_pid_initialized_{false};
  double depth_integral_{0.0};
  double depth_previous_error_{0.0};
  rclcpp::Time depth_pid_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    depth_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_frame_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_search_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_granted_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr physical_detach_count_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr fcu_state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_monitor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_confirmed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_event_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace auv_lane_vision_control

// 통신을 초기화하고 제어 노드를 실행한 뒤 종료 시 채널을 해제한다.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auv_lane_vision_control::LaneVisionControllerNode>();
  rclcpp::spin(node);
  node->publish_release_once();
  rclcpp::shutdown();
  return 0;
}
