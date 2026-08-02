#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <auv_msg/msg/collector_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "auv_lane_vision_control/arena_frame_transform.hpp"
#include "auv_lane_vision_control/depth_p_controller.hpp"
#include "auv_lane_vision_control/dump_cycle.hpp"
#include "auv_lane_vision_control/lane_planner.hpp"

namespace auv_lane_vision_control
{
namespace
{
uint32_t parse_uint_field(const std::string & text, const std::string & key)
{
  const auto begin = text.find(key + "=");
  if (begin == std::string::npos) {
    return 0;
  }
  return static_cast<uint32_t>(std::stoul(text.substr(begin + key.size() + 1)));
}

double parse_double_field(const std::string & text, const std::string & key, double fallback)
{
  const auto begin = text.find(key + "=");
  if (begin == std::string::npos) {
    return fallback;
  }
  return std::stod(text.substr(begin + key.size() + 1));
}
}  // namespace

class SurfaceBuoyMissionNode : public rclcpp::Node
{
public:
  SurfaceBuoyMissionNode()
  : Node("surface_buoy_mission_node")
  {
    declare_parameters();
    create_interfaces();
    state_entered_at_ = now();
    publish_state();
    publish_counts();
    RCLCPP_INFO(
      get_logger(),
      "Surface mission ready: front=%s top=%s score=(%.3f, %.3f), camera contract=1280x720@10Hz",
      front_bbox_topic_.c_str(), top_bbox_topic_.c_str(), bonus_center_.x, bonus_center_.y);
  }

  void publish_release_once()
  {
    if (!active_ || release_sent_) {
      return;
    }
    auto channels = nochange_channels();
    release_channels(channels);
    publish_channels(channels);
    release_sent_ = true;
  }

private:
  enum class State
  {
    IDLE,
    ASCEND_TO_COLLECTION_DEPTH,
    INITIAL_TOP_CHECK,
    SURFACE_SEARCH,
    SURFACE_ALIGN,
    SURFACE_CAPTURE,
    MOVE_TO_BONUS,
    DESCEND_TO_DUMP_DEPTH,
    MOVE_TO_BONUS_CENTER,
    DUMP_EJECT,
    DUMP_CHECK,
    RETURN_TO_BONUS_CENTER,
    RETURN_TO_WORK_DEPTH,
    FAILSAFE
  };

  struct Detection
  {
    double cx{0.0};
    double cy{0.0};
    double width{0.0};
    double height{0.0};
    double image_width{0.0};
    double image_height{0.0};
    double confidence{0.0};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
  };

  void declare_parameters()
  {
    front_bbox_topic_ = declare_parameter<std::string>(
      "surface_front_bbox_topic", "/vision/surface/front/buoy_bbox");
    top_bbox_topic_ = declare_parameter<std::string>(
      "surface_top_bbox_topic", "/vision/surface/top/buoy_bbox");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/homing/sim_odom");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    start_frame_topic_ = declare_parameter<std::string>("start_frame_topic", "/start_frame");
    surface_start_topic_ = declare_parameter<std::string>("surface_start_topic", "/mission/surface_start");
    surface_complete_topic_ = declare_parameter<std::string>("surface_complete_topic", "/mission/surface_complete");
    arena_config_topic_ = declare_parameter<std::string>("arena_config_topic", "/mission/arena_config");
    score_release_topic_ = declare_parameter<std::string>("score_release_topic", "/mission/score_release");
    rc_topic_ = declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");

    bonus_center_.x = declare_parameter<double>("bonus_zone_center_x_m", 9.081);
    bonus_center_.y = declare_parameter<double>("bonus_zone_center_y_m", -1.305);
    bonus_radius_m_ = declare_parameter<double>("bonus_zone_radius_m", 0.65);
    bonus_approach_distance_m_ = declare_parameter<double>("bonus_approach_distance_m", 1.0);
    bonus_dump_heading_rad_ = declare_parameter<double>("bonus_dump_heading_rad", 0.0);
    score_zone_world_z_m_ = declare_parameter<double>("score_zone_world_z_m", -0.30);
    collection_depth_m_ = declare_parameter<double>("collection_depth_m", 0.30);
    dump_depth_m_ = declare_parameter<double>("dump_depth_m", 0.85);
    depth_tolerance_m_ = declare_parameter<double>("depth_tolerance_m", 0.12);

    batch_capacity_ = declare_parameter<int>("batch_capacity", 3);
    surface_total_buoy_count_ = declare_parameter<int>("surface_total_buoy_count", 5);
    max_dump_attempts_ = declare_parameter<int>("max_dump_attempts", 3);
    dump_exit_radius_m_ = declare_parameter<double>("dump_exit_radius_m", 0.70);
    dump_motion_timeout_s_ = declare_parameter<double>("dump_motion_timeout_sec", 4.0);
    dump_settle_s_ = declare_parameter<double>("dump_settle_sec", 0.7);
    capture_forward_s_ = declare_parameter<double>("capture_forward_sec", 2.0);
    capture_ignore_s_ = declare_parameter<double>("capture_ignore_sec", 0.8);

    top_roi_x_min_ = declare_parameter<double>("top_net_roi_x_min", 0.10);
    top_roi_x_max_ = declare_parameter<double>("top_net_roi_x_max", 0.90);
    top_roi_y_min_ = declare_parameter<double>("top_net_roi_y_min", 0.05);
    top_roi_y_max_ = declare_parameter<double>("top_net_roi_y_max", 0.95);
    top_confirm_s_ = declare_parameter<double>("top_occupied_confirm_sec", 0.5);
    top_empty_confirm_s_ = declare_parameter<double>("top_empty_confirm_sec", 0.5);
    detection_timeout_s_ = declare_parameter<double>("top_detection_timeout_sec", 0.7);
    align_deadband_x_ = declare_parameter<double>("surface_align_deadband_x", 0.07);
    align_stable_s_ = declare_parameter<double>("surface_align_stable_sec", 0.4);
    capture_bottom_ratio_ = declare_parameter<double>("surface_capture_bottom_ratio", 0.82);

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    depth_kp_pwm_per_m_ = declare_parameter<double>("surface_depth_kp_pwm_per_m", 130.0);
    max_depth_delta_pwm_ = declare_parameter<int>("surface_max_depth_delta_pwm", 180);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    search_forward_pwm_ = declare_parameter<int>("surface_search_forward_pwm", 1600);
    capture_forward_pwm_ = declare_parameter<int>("surface_capture_forward_pwm", 1650);
    dump_forward_pwm_ = declare_parameter<int>("dump_forward_pwm", 1660);
    dump_reverse_pwm_ = declare_parameter<int>("dump_reverse_pwm", 1340);
    max_yaw_delta_pwm_ = declare_parameter<int>("surface_max_yaw_delta_pwm", 150);
    waypoint_tolerance_m_ = declare_parameter<double>("surface_waypoint_tolerance_m", 0.25);
    heading_tolerance_rad_ = declare_parameter<double>("surface_heading_tolerance_rad", 0.20);
    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);

    if (batch_capacity_ < 1 || batch_capacity_ > 4) {
      throw std::invalid_argument("batch_capacity must be in [1, 4]");
    }
  }

  void create_interfaces()
  {
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    front_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      front_bbox_topic_, 10, std::bind(&SurfaceBuoyMissionNode::on_front_bbox, this, std::placeholders::_1));
    top_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      top_bbox_topic_, 10, std::bind(&SurfaceBuoyMissionNode::on_top_bbox, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 30, std::bind(&SurfaceBuoyMissionNode::on_odometry, this, std::placeholders::_1));
    depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      depth_pose_topic_, 10, std::bind(&SurfaceBuoyMissionNode::on_depth, this, std::placeholders::_1));
    start_frame_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      start_frame_topic_, latched, std::bind(&SurfaceBuoyMissionNode::on_start_frame, this, std::placeholders::_1));
    arena_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      arena_config_topic_, latched, std::bind(&SurfaceBuoyMissionNode::on_arena_config, this, std::placeholders::_1));
    start_sub_ = create_subscription<std_msgs::msg::String>(
      surface_start_topic_, latched, std::bind(&SurfaceBuoyMissionNode::on_surface_start, this, std::placeholders::_1));
    collector_sub_ = create_subscription<auv_msg::msg::CollectorState>(
      "/collector/state", 30, std::bind(&SurfaceBuoyMissionNode::on_collector_state, this, std::placeholders::_1));

    state_pub_ = create_publisher<std_msgs::msg::String>("/mission/surface_state", latched);
    complete_pub_ = create_publisher<std_msgs::msg::String>(surface_complete_topic_, 10);
    score_release_pub_ = create_publisher<std_msgs::msg::String>(score_release_topic_, 10);
    remaining_pub_ = create_publisher<std_msgs::msg::UInt32>("/mission/surface_remaining_count", latched);
    deposit_pub_ = create_publisher<std_msgs::msg::UInt32>("/mission/bonus_deposit_count", latched);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)),
      std::bind(&SurfaceBuoyMissionNode::on_timer, this));
  }

  void on_arena_config(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 7 || lane_planner_) {
      return;
    }
    ArenaConfig config;
    config.length_m = msg->data[0];
    config.width_m = msg->data[1];
    config.offset_x_m = msg->data[2];
    config.offset_y_m = msg->data[3];
    config.safety_margin_m = msg->data[4];
    config.lane_search_offset_m = msg->data[5];
    config.start_corner = msg->data[6] > 0.5 ? "bottom_right" : "bottom_left";
    lane_planner_ = std::make_unique<LanePlanner>(config);
    const auto & bounds = lane_planner_->safe_bounds();
    if (
      bonus_center_.x - bonus_radius_m_ < bounds.x_min ||
      bonus_center_.x + bonus_radius_m_ > bounds.x_max ||
      bonus_center_.y - bonus_radius_m_ < bounds.y_min ||
      bonus_center_.y + bonus_radius_m_ > bounds.y_max)
    {
      throw std::invalid_argument("bonus-zone circle is outside the shared arena safe bounds");
    }
    surface_lane_completed_.assign(lane_planner_->lanes().size(), false);
  }

  void on_start_frame(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (arena_transform_.initialized()) {
      return;
    }
    const double yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
    arena_transform_.initialize({msg->pose.position.x, msg->pose.position.y}, yaw);
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!arena_transform_.initialized()) {
      return;
    }
    current_position_ = arena_transform_.position_from_odom(
      {msg->pose.pose.position.x, msg->pose.pose.position.y});
    const double odom_yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    current_yaw_rad_ = arena_transform_.yaw_from_odom(odom_yaw);
    have_odometry_ = true;
    odometry_at_ = now();
  }

  void on_depth(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const double depth = -msg->pose.pose.position.z;
    if (std::isfinite(depth) && depth >= 0.0) {
      depth_m_ = depth;
      depth_at_ = now();
    }
  }

  std::vector<Detection> detections(const std_msgs::msg::Float32MultiArray & msg) const
  {
    std::vector<Detection> rows;
    for (std::size_t base = 0; base + 9 < msg.data.size(); base += 10) {
      if (
        msg.data[base + 1] < 0.5F ||
        static_cast<int>(std::lround(msg.data[base + 2])) != buoy_class_id_ ||
        msg.data[base + 8] <= 0.0F || msg.data[base + 9] <= 0.0F)
      {
        continue;
      }
      rows.push_back(
        {msg.data[base + 4], msg.data[base + 5], msg.data[base + 6], msg.data[base + 7],
          msg.data[base + 8], msg.data[base + 9], msg.data[base + 3], now()});
    }
    return rows;
  }

  void on_front_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto rows = detections(*msg);
    if (rows.empty()) {
      front_.reset();
      return;
    }
    front_ = *std::max_element(
      rows.begin(), rows.end(), [](const Detection & a, const Detection & b) {
        return a.width * a.height < b.width * b.height;
      });
  }

  void on_top_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto rows = detections(*msg);
    bool occupied = false;
    for (const auto & row : rows) {
      const double x0 = (row.cx - 0.5 * row.width) / row.image_width;
      const double x1 = (row.cx + 0.5 * row.width) / row.image_width;
      const double y0 = (row.cy - 0.5 * row.height) / row.image_height;
      const double y1 = (row.cy + 0.5 * row.height) / row.image_height;
      if (state_ == State::INITIAL_TOP_CHECK ||
        (x1 >= top_roi_x_min_ && x0 <= top_roi_x_max_ &&
        y1 >= top_roi_y_min_ && y0 <= top_roi_y_max_))
      {
        occupied = true;
        break;
      }
    }
    top_occupied_ = occupied;
    top_received_at_ = now();
    if (occupied) {
      if (!top_occupied_since_) {
        top_occupied_since_ = now();
      }
      top_empty_since_.reset();
    } else {
      if (!top_empty_since_) {
        top_empty_since_ = now();
      }
      top_occupied_since_.reset();
    }
  }

  void on_collector_state(const auv_msg::msg::CollectorState::SharedPtr msg)
  {
    if (msg->netted && !msg->target_id.empty() && captured_ids_.insert(msg->target_id).second) {
      if (active_) {
        ++batch_capture_count_;
        capture_confirmed_ = true;
      }
      publish_counts();
    }
    if (msg->released && !msg->target_id.empty() && deposited_ids_.insert(msg->target_id).second) {
      publish_counts();
    }
  }

  void on_surface_start(const std_msgs::msg::String::SharedPtr msg)
  {
    const uint32_t cycle = parse_uint_field(msg->data, "cycle");
    if (cycle == 0 || cycle <= last_completed_cycle_ || cycle == cycle_id_) {
      return;
    }
    if (!lane_planner_ || !arena_transform_.initialized()) {
      RCLCPP_WARN(get_logger(), "Surface start deferred: shared arena/start frame unavailable");
      pending_start_ = msg->data;
      return;
    }
    begin_cycle(msg->data);
  }

  void begin_cycle(const std::string & contract)
  {
    cycle_id_ = parse_uint_field(contract, "cycle");
    final_cycle_ = contract.find("final=true") != std::string::npos;
    work_depth_m_ = parse_double_field(contract, "work_depth", dump_depth_m_);
    batch_capture_count_ = 0;
    dump_attempt_count_ = 0;
    surface_lane_completed_.assign(lane_planner_->lanes().size(), false);
    active_surface_lane_.reset();
    front_.reset();
    capture_confirmed_ = false;
    release_sent_ = false;
    active_ = true;
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_topic_, 10);
    close_score_gate();
    transition(State::ASCEND_TO_COLLECTION_DEPTH, "lane RC handoff received");
  }

  void on_timer()
  {
    if (!pending_start_.empty() && lane_planner_ && arena_transform_.initialized()) {
      const std::string pending = pending_start_;
      pending_start_.clear();
      begin_cycle(pending);
    }
    if (!active_) {
      return;
    }
    if (!have_odometry_ || !depth_m_) {
      return;
    }
    auto channels = nochange_channels();
    set_neutral(channels);
    switch (state_) {
      case State::IDLE: return;
      case State::ASCEND_TO_COLLECTION_DEPTH: run_ascend(channels); break;
      case State::INITIAL_TOP_CHECK: run_initial_top_check(channels); break;
      case State::SURFACE_SEARCH: run_search(channels); break;
      case State::SURFACE_ALIGN: run_align(channels); break;
      case State::SURFACE_CAPTURE: run_capture(channels); break;
      case State::MOVE_TO_BONUS: run_move_bonus(channels); break;
      case State::DESCEND_TO_DUMP_DEPTH: run_descend_dump(channels); break;
      case State::MOVE_TO_BONUS_CENTER: run_bonus_center(channels); break;
      case State::DUMP_EJECT: run_dump_eject(channels); break;
      case State::DUMP_CHECK: run_dump_check(channels); break;
      case State::RETURN_TO_BONUS_CENTER: run_return_center(channels); break;
      case State::RETURN_TO_WORK_DEPTH: run_return_depth(channels); break;
      case State::FAILSAFE: fail_cycle("surface failsafe"); return;
    }
    publish_channels(channels);
  }

  void run_ascend(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    if (std::abs(*depth_m_ - collection_depth_m_) <= depth_tolerance_m_) {
      if (!initial_top_check_done_) {
        transition(State::INITIAL_TOP_CHECK, "collection depth reached");
      } else {
        begin_search("collection depth reached");
      }
    }
  }

  void run_initial_top_check(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    if (recent_top() || state_age() >= detection_timeout_s_) {
      initial_top_check_done_ = true;
      begin_search(top_occupied_ ? "initial top buoy visible" : "initial top check complete");
    }
  }

  void begin_search(const std::string & reason)
  {
    select_surface_lane();
    if (!active_surface_lane_) {
      finish_search();
      return;
    }
    transition(State::SURFACE_SEARCH, reason);
  }

  void select_surface_lane()
  {
    const auto choice = lane_planner_->closest_uncompleted_endpoint(
      current_position_, surface_lane_completed_);
    if (!choice) {
      active_surface_lane_.reset();
      return;
    }
    active_surface_lane_ = choice->lane_index;
    surface_lane_start_ = choice->start;
    surface_lane_finish_ = choice->finish;
    surface_heading_to_start_ = true;
  }

  void run_search(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    if (recent_front() && state_age() >= capture_ignore_s_) {
      align_started_at_.reset();
      transition(State::SURFACE_ALIGN, "front buoy acquired");
      return;
    }
    if (!active_surface_lane_) {
      finish_search();
      return;
    }
    const Vec2 target = surface_heading_to_start_ ? surface_lane_start_ : surface_lane_finish_;
    if (follow_waypoint(channels, target, search_forward_pwm_, collection_depth_m_)) {
      if (surface_heading_to_start_) {
        surface_heading_to_start_ = false;
      } else {
        surface_lane_completed_[*active_surface_lane_] = true;
        select_surface_lane();
        if (!active_surface_lane_) {
          finish_search();
        }
      }
    }
  }

  void run_align(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    if (!recent_front()) {
      begin_search("front target lost");
      return;
    }
    const double nx = front_->cx / front_->image_width;
    const double error = 0.5 - nx;
    const int yaw_delta = static_cast<int>(std::lround(std::clamp(380.0 * error, -1.0 * max_yaw_delta_pwm_, 1.0 * max_yaw_delta_pwm_)));
    set_channel(channels, 4, neutral_pwm_ + yaw_delta);
    if (std::abs(error) <= align_deadband_x_) {
      if (!align_started_at_) {
        align_started_at_ = now();
      }
      set_channel(channels, 5, capture_forward_pwm_);
      const double bbox_bottom = (front_->cy + 0.5 * front_->height) / front_->image_height;
      if (
        (now() - *align_started_at_).seconds() >= align_stable_s_ &&
        bbox_bottom >= capture_bottom_ratio_)
      {
        capture_confirmed_ = false;
        capture_front_start_y_ = front_->cy / front_->image_height;
        capture_front_min_y_ = capture_front_start_y_;
        transition(State::SURFACE_CAPTURE, "front centerline stable");
      }
    } else {
      align_started_at_.reset();
    }
  }

  void run_capture(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    set_channel(channels, 5, capture_forward_pwm_);
    if (recent_front()) {
      capture_front_min_y_ = std::min(capture_front_min_y_, front_->cy / front_->image_height);
    }
    const bool front_moved_up = capture_front_start_y_ - capture_front_min_y_ >= 0.08;
    const bool transitioned_to_top = front_moved_up && top_occupied_confirmed();
    if (capture_confirmed_ || transitioned_to_top) {
      if (!capture_confirmed_) {
        ++batch_capture_count_;
        publish_counts();
      }
      capture_confirmed_ = false;
      if (batch_capture_count_ >= static_cast<uint32_t>(batch_capacity_)) {
        transition(State::MOVE_TO_BONUS, "batch capacity reached");
      } else {
        begin_search("capture committed by front-to-top transition");
      }
      return;
    }
    if (state_age() >= capture_forward_s_) {
      begin_search("capture not confirmed; resume surface lanes");
    }
  }

  void finish_search()
  {
    if (batch_capture_count_ > 0) {
      transition(State::MOVE_TO_BONUS, "all surface lanes searched");
    } else {
      transition(State::RETURN_TO_WORK_DEPTH, "surface lanes empty");
    }
  }

  void run_move_bonus(std::array<uint16_t, 18> & channels)
  {
    if (distance(current_position_, bonus_center_) <= bonus_approach_distance_m_) {
      transition(State::DESCEND_TO_DUMP_DEPTH, "bonus approach radius reached");
      return;
    }
    follow_waypoint(channels, bonus_center_, search_forward_pwm_, collection_depth_m_);
  }

  void run_descend_dump(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, dump_depth_m_);
    if (std::abs(*depth_m_ - dump_depth_m_) <= depth_tolerance_m_) {
      transition(State::MOVE_TO_BONUS_CENTER, "dump depth reached");
    }
  }

  void run_bonus_center(std::array<uint16_t, 18> & channels)
  {
    if (follow_waypoint(channels, bonus_center_, search_forward_pwm_, dump_depth_m_)) {
      const double heading_error = wrap_pi(bonus_dump_heading_rad_ - current_yaw_rad_);
      set_channel(channels, 4, neutral_pwm_ + static_cast<int>(std::lround(
        std::clamp(350.0 * heading_error, -1.0 * max_yaw_delta_pwm_, 1.0 * max_yaw_delta_pwm_))));
      if (std::abs(heading_error) <= heading_tolerance_rad_) {
        start_dump_attempt();
      }
    }
  }

  void start_dump_attempt()
  {
    ++dump_attempt_count_;
    open_score_gate();
    transition(State::DUMP_EJECT, "dump attempt " + std::to_string(dump_attempt_count_));
  }

  void run_dump_eject(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, dump_depth_m_);
    const bool forward = dump_attempt_count_ == 1 || dump_attempt_count_ == 3;
    set_channel(channels, 5, forward ? dump_forward_pwm_ : dump_reverse_pwm_);
    if (distance(current_position_, bonus_center_) >= dump_exit_radius_m_ || state_age() >= dump_motion_timeout_s_) {
      close_score_gate();
      transition(State::DUMP_CHECK, "dump exit reached");
    }
  }

  void run_dump_check(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, dump_depth_m_);
    if (state_age() < dump_settle_s_) {
      return;
    }
    TopNetStatus top_status = TopNetStatus::OCCUPIED;
    if (!recent_top()) {
      top_status = TopNetStatus::STALE;
      RCLCPP_WARN(get_logger(), "Top camera stale/unknown after dump; ending dump loop");
    } else if (top_empty_confirmed() || !top_occupied_) {
      top_status = TopNetStatus::EMPTY;
    }
    if (!should_repeat_dump(
        top_status, dump_attempt_count_, static_cast<uint32_t>(max_dump_attempts_)))
    {
      batch_capture_count_ = 0;
      transition(
        State::RETURN_TO_WORK_DEPTH,
        top_status == TopNetStatus::STALE ? "top stale after dump" :
        top_status == TopNetStatus::EMPTY ? "collector empty after dump" :
        "maximum dump attempts reached");
      return;
    }
    transition(State::RETURN_TO_BONUS_CENTER, "collector still occupied");
  }

  void run_return_center(std::array<uint16_t, 18> & channels)
  {
    if (follow_waypoint(channels, bonus_center_, search_forward_pwm_, dump_depth_m_)) {
      transition(State::MOVE_TO_BONUS_CENTER, "bonus center reacquired");
    }
  }

  void run_return_depth(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, work_depth_m_);
    if (std::abs(*depth_m_ - work_depth_m_) <= depth_tolerance_m_) {
      complete_cycle();
    }
  }

  bool follow_waypoint(
    std::array<uint16_t, 18> & channels, const Vec2 & target,
    int forward_pwm, double target_depth)
  {
    hold_depth(channels, target_depth);
    const Vec2 delta = target - current_position_;
    if (norm(delta) <= waypoint_tolerance_m_) {
      return true;
    }
    const double desired = std::atan2(delta.y, delta.x);
    const double error = wrap_pi(desired - current_yaw_rad_);
    const int yaw_delta = static_cast<int>(std::lround(
      std::clamp(350.0 * error, -1.0 * max_yaw_delta_pwm_, 1.0 * max_yaw_delta_pwm_)));
    set_channel(channels, 4, neutral_pwm_ + yaw_delta);
    if (std::abs(error) <= heading_tolerance_rad_) {
      set_channel(channels, 5, forward_pwm);
    }
    return false;
  }

  void hold_depth(std::array<uint16_t, 18> & channels, double target_depth)
  {
    set_channel(
      channels, 3,
      depth_p_pwm(
        target_depth, *depth_m_,
        DepthPConfig{depth_kp_pwm_per_m_, max_depth_delta_pwm_, neutral_pwm_, true}));
  }

  bool recent_front() const
  {
    return front_ && (now() - front_->received).seconds() <= detection_timeout_s_;
  }

  bool recent_top() const
  {
    return top_received_at_.nanoseconds() > 0 &&
      (now() - top_received_at_).seconds() <= detection_timeout_s_;
  }

  bool top_occupied_confirmed() const
  {
    return recent_top() && top_occupied_ && top_occupied_since_ &&
      (now() - *top_occupied_since_).seconds() >= top_confirm_s_;
  }

  bool top_empty_confirmed() const
  {
    return recent_top() && !top_occupied_ && top_empty_since_ &&
      (now() - *top_empty_since_).seconds() >= top_empty_confirm_s_;
  }

  void open_score_gate()
  {
    const Vec2 world = arena_transform_.position_to_odom(bonus_center_);
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"state\":\"RELEASE\",\"score_zone\":{\"xyz\":[" <<
      world.x << ',' << world.y << ',' << score_zone_world_z_m_ << "]}}";
    msg.data = out.str();
    score_release_pub_->publish(msg);
  }

  void close_score_gate()
  {
    std_msgs::msg::String msg;
    msg.data = "{\"state\":\"IDLE\",\"score_zone\":{\"xyz\":[0,0,0]}}";
    score_release_pub_->publish(msg);
  }

  void complete_cycle()
  {
    close_score_gate();
    publish_release_once();
    rc_pub_.reset();
    std_msgs::msg::String msg;
    msg.data = "cycle=" + std::to_string(cycle_id_) + ";success=true";
    complete_pub_->publish(msg);
    last_completed_cycle_ = cycle_id_;
    active_ = false;
    state_ = State::IDLE;
    publish_state();
    RCLCPP_INFO(
      get_logger(), "Surface cycle %u complete (final=%s, deposited=%zu, remaining=%u)",
      cycle_id_, final_cycle_ ? "true" : "false", deposited_ids_.size(), remaining_count());
  }

  void fail_cycle(const std::string & reason)
  {
    close_score_gate();
    publish_release_once();
    rc_pub_.reset();
    std_msgs::msg::String msg;
    msg.data = "cycle=" + std::to_string(cycle_id_) + ";success=false;reason=" + reason;
    complete_pub_->publish(msg);
    active_ = false;
  }

  void transition(State next, const std::string & reason)
  {
    RCLCPP_INFO(get_logger(), "Surface %s -> %s: %s", state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    publish_state();
  }

  double state_age() const {return (now() - state_entered_at_).seconds();}

  static const char * state_name(State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::ASCEND_TO_COLLECTION_DEPTH: return "ASCEND_TO_COLLECTION_DEPTH";
      case State::INITIAL_TOP_CHECK: return "INITIAL_TOP_CHECK";
      case State::SURFACE_SEARCH: return "SURFACE_SEARCH";
      case State::SURFACE_ALIGN: return "SURFACE_ALIGN";
      case State::SURFACE_CAPTURE: return "SURFACE_CAPTURE";
      case State::MOVE_TO_BONUS: return "MOVE_TO_BONUS";
      case State::DESCEND_TO_DUMP_DEPTH: return "DESCEND_TO_DUMP_DEPTH";
      case State::MOVE_TO_BONUS_CENTER: return "MOVE_TO_BONUS_CENTER";
      case State::DUMP_EJECT: return "DUMP_EJECT";
      case State::DUMP_CHECK: return "DUMP_CHECK";
      case State::RETURN_TO_BONUS_CENTER: return "RETURN_TO_BONUS_CENTER";
      case State::RETURN_TO_WORK_DEPTH: return "RETURN_TO_WORK_DEPTH";
      case State::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  void publish_state()
  {
    if (!state_pub_) {return;}
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    state_pub_->publish(msg);
  }

  uint32_t remaining_count() const
  {
    return static_cast<uint32_t>(std::max<int>(0, surface_total_buoy_count_ - static_cast<int>(captured_ids_.size())));
  }

  void publish_counts()
  {
    if (!remaining_pub_ || !deposit_pub_) {return;}
    std_msgs::msg::UInt32 remaining;
    remaining.data = remaining_count();
    remaining_pub_->publish(remaining);
    std_msgs::msg::UInt32 deposited;
    deposited.data = static_cast<uint32_t>(deposited_ids_.size());
    deposit_pub_->publish(deposited);
  }

  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  void set_neutral(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, 3, neutral_pwm_);
    set_channel(channels, 4, neutral_pwm_);
    set_channel(channels, 5, neutral_pwm_);
  }

  void release_channels(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, 3, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, 4, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, 5, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  void set_channel(std::array<uint16_t, 18> & channels, int channel, int pwm) const
  {
    if (pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE && pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE) {
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<std::size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    if (!rc_pub_) {
      return;
    }
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels = channels;
    rc_pub_->publish(msg);
  }

  std::string front_bbox_topic_, top_bbox_topic_, odometry_topic_, depth_pose_topic_;
  std::string start_frame_topic_, surface_start_topic_, surface_complete_topic_;
  std::string arena_config_topic_, score_release_topic_, rc_topic_;
  Vec2 bonus_center_{};
  double bonus_radius_m_{0.65}, bonus_approach_distance_m_{1.0}, bonus_dump_heading_rad_{0.0};
  double score_zone_world_z_m_{-0.3}, collection_depth_m_{0.30}, dump_depth_m_{0.85};
  double work_depth_m_{0.85}, depth_tolerance_m_{0.12};
  int batch_capacity_{3}, surface_total_buoy_count_{5}, max_dump_attempts_{3};
  double dump_exit_radius_m_{0.70}, dump_motion_timeout_s_{4.0}, dump_settle_s_{0.7};
  double capture_forward_s_{2.0}, capture_ignore_s_{0.8};
  double top_roi_x_min_{0.1}, top_roi_x_max_{0.9}, top_roi_y_min_{0.05}, top_roi_y_max_{0.95};
  double top_confirm_s_{0.5}, top_empty_confirm_s_{0.5}, detection_timeout_s_{0.7};
  double align_deadband_x_{0.07}, align_stable_s_{0.4}, capture_bottom_ratio_{0.82};
  double control_rate_hz_{20.0}, depth_kp_pwm_per_m_{130.0}, waypoint_tolerance_m_{0.25};
  double heading_tolerance_rad_{0.2};
  int max_depth_delta_pwm_{180}, neutral_pwm_{1500}, min_pwm_{1300}, max_pwm_{1700};
  int search_forward_pwm_{1600}, capture_forward_pwm_{1650};
  int dump_forward_pwm_{1660}, dump_reverse_pwm_{1340}, max_yaw_delta_pwm_{150};
  int buoy_class_id_{0};

  State state_{State::IDLE};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  ArenaFrameTransform arena_transform_;
  std::unique_ptr<LanePlanner> lane_planner_;
  std::vector<bool> surface_lane_completed_;
  std::optional<std::size_t> active_surface_lane_;
  Vec2 surface_lane_start_{}, surface_lane_finish_{}, current_position_{};
  bool surface_heading_to_start_{true};
  double current_yaw_rad_{0.0};
  bool have_odometry_{false};
  rclcpp::Time odometry_at_{0, 0, RCL_ROS_TIME};
  std::optional<double> depth_m_;
  rclcpp::Time depth_at_{0, 0, RCL_ROS_TIME};
  std::optional<Detection> front_;
  bool top_occupied_{false};
  rclcpp::Time top_received_at_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> top_occupied_since_, top_empty_since_, align_started_at_;
  bool initial_top_check_done_{false}, capture_confirmed_{false};
  double capture_front_start_y_{0.0}, capture_front_min_y_{0.0};
  uint32_t batch_capture_count_{0}, dump_attempt_count_{0}, cycle_id_{0}, last_completed_cycle_{0};
  bool final_cycle_{false}, active_{false}, release_sent_{false};
  std::string pending_start_;
  std::set<std::string> captured_ids_, deposited_ids_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr front_sub_, top_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_frame_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arena_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
  rclcpp::Subscription<auv_msg::msg::CollectorState>::SharedPtr collector_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, complete_pub_, score_release_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr remaining_pub_, deposit_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace auv_lane_vision_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auv_lane_vision_control::SurfaceBuoyMissionNode>();
  rclcpp::spin(node);
  node->publish_release_once();
  rclcpp::shutdown();
  return 0;
}
